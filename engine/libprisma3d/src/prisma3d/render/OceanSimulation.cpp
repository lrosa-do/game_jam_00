#include "prisma3d/render/OceanSimulation.h"
#include "prisma3d/render/ShaderVariants.h"


#include "perlin.h"
#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const float kHalfSqrt2 = 0.7071068f;
const float kGravity = 981.0f;
const float kPi = 3.14159265358979323846f;
const double kTwoPi = 6.283185307179586476925286766559;
const std::uint32_t kTileSize = 8;
const std::uint32_t kFftGranularity = 128;
const std::uint32_t kFftSlices = 2;
const std::uint32_t kFftPasses = 6;

struct OceanParamsGpu
{
    std::uint32_t dims[4];
    std::uint32_t offsets[4];
    float values[4];
};

struct FftParamsGpu
{
    std::uint32_t strides[4];
    float phase[4];
};

static_assert(sizeof(OceanParamsGpu) == 48, "OceanParamsGpu must match the std140 block");
static_assert(sizeof(FftParamsGpu) == 32, "FftParamsGpu must match the std140 block");

struct Random
{
    std::uint32_t state;

    float uniform()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<float>(state >> 8) * (1.0f / 16777215.0f);
    }

    float gauss()
    {
        float u1 = uniform();
        const float u2 = uniform();
        if (u1 < 1e-6f) u1 = 1e-6f;
        return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * kPi * u2);
    }
};

float phillips(const Math::Vec2& k, const Math::Vec2& wind, float v, float a, float dirDepend)
{
    const float l = v * v / kGravity;
    const float w = l / 1000.0f;

    const float ksqr = k.x * k.x + k.y * k.y;
    const float kcos = k.x * wind.x + k.y * wind.y;
    float value = a * expf(-1.0f / (l * l * ksqr)) / (ksqr * ksqr * ksqr) * (kcos * kcos);

    if (kcos < 0.0f) value *= dirDepend;

    return value * expf(-ksqr * w * w);
}

std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment)
{
    return (value + alignment - 1) / alignment * alignment;
}

} // namespace

struct OceanSimulation::Gpu
{
    prisma::Driver* driver = nullptr;
    prisma::ShaderHandle shaders[5];
    prisma::PipelineHandle simulate;
    prisma::PipelineHandle fft;
    prisma::PipelineHandle fftLast;
    prisma::PipelineHandle displacement;
    prisma::PipelineHandle gradient;
    prisma::SamplerHandle sampler;
    prisma::BufferHandle h0;
    prisma::BufferHandle omega;
    prisma::BufferHandle ht;
    prisma::BufferHandle dxyz;
    prisma::BufferHandle temp;
    prisma::BufferHandle params;
    prisma::BufferHandle fftParams;
    std::uint32_t fftStride = 0;
    TextureId displacementMap;
    TextureId gradientMap;
    TextureId perlinMap;
};

OceanSimulation::OceanSimulation(TextureStore& textures)
        : mTextures(textures),
          mParams(),
          mGpu(nullptr),
          mTime(0.0f)
{
}

OceanSimulation::~OceanSimulation()
{
    release();
}

bool OceanSimulation::supported(prisma::Driver* driver)
{
    return driver && driver->caps().compute;
}

void OceanSimulation::initialSpectrum(const OceanSimulationParams& params,
        ct::Vector<Math::Vec2>& h0, ct::Vector<float>& omega)
{
    h0.clear();
    omega.clear();
    h0.resize(kInputCount, Math::Vec2(0.0f, 0.0f));
    omega.resize(kInputCount, 0.0f);

    Math::Vec2 wind = params.windDirection;
    const float windLength = sqrtf(wind.x * wind.x + wind.y * wind.y);
    if (windLength > 0.0f) wind = Math::Vec2(wind.x / windLength, wind.y / windLength);
    const float a = params.waveAmplitude * 1e-7f;
    const float v = params.windSpeed;
    const float dirDepend = params.windDependency;
    const int dim = static_cast<int>(kSize);
    const float patchLength = params.patchLength;

    Random random;
    random.state = params.seed ? params.seed : 1u;

    for (int i = 0; i <= dim; i++)
    {
        Math::Vec2 k;
        k.y = (-static_cast<float>(dim) / 2.0f + static_cast<float>(i)) *
              (2.0f * kPi / patchLength);

        for (int j = 0; j <= dim; j++)
        {
            k.x = (-static_cast<float>(dim) / 2.0f + static_cast<float>(j)) *
                  (2.0f * kPi / patchLength);

            const float phil =
                    (k.x == 0.0f && k.y == 0.0f) ? 0.0f : sqrtf(phillips(k, wind, v, a, dirDepend));

            const std::size_t index =
                    static_cast<std::size_t>(i) * kInputWidth + static_cast<std::size_t>(j);
            h0[index].x = phil * random.gauss() * kHalfSqrt2;
            h0[index].y = phil * random.gauss() * kHalfSqrt2;
            omega[index] = sqrtf(kGravity * sqrtf(k.x * k.x + k.y * k.y));
        }
    }
}

bool OceanSimulation::create(const OceanSimulationParams& params)
{
    release();
    mParams = params;
    prisma::Driver* driver = mTextures.driver();
    if (!supported(driver)) return false;

    mGpu = new Gpu();
    Gpu& gpu = *mGpu;
    gpu.driver = driver;
    const prisma::Caps& caps = driver->caps();

    gpu.shaders[0] = driver->createShader(prisma3d::shaderDesc(ocean_simulate_comp, caps));
    gpu.shaders[1] = driver->createShader(prisma3d::shaderDesc(ocean_fft_comp, caps));
    gpu.shaders[2] = driver->createShader(prisma3d::shaderDesc(ocean_fft_v2_comp, caps));
    gpu.shaders[3] = driver->createShader(prisma3d::shaderDesc(ocean_displacement_comp, caps));
    gpu.shaders[4] = driver->createShader(prisma3d::shaderDesc(ocean_gradient_comp, caps));
    for (int i = 0; i < 5; ++i)
        if (!gpu.shaders[i].valid())
        {
            log_error("ocean: could not create the compute shaders");
            release();
            return false;
        }

    {
        prisma::ComputePipelineDesc desc;
        desc.shader = gpu.shaders[0];
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "OceanParams";
        desc.uniformBlocks[0].slot = 0;
        desc.storageBufferCount = 3;
        desc.storageBuffers[0].name = "InputH0";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "InputOmega";
        desc.storageBuffers[1].slot = 1;
        desc.storageBuffers[2].name = "OutputHt";
        desc.storageBuffers[2].slot = 2;
        desc.debugName = "ocean simulate";
        gpu.simulate = driver->createComputePipeline(desc);
    }
    for (int i = 0; i < 2; ++i)
    {
        prisma::ComputePipelineDesc desc;
        desc.shader = gpu.shaders[1 + i];
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "FftParams";
        desc.uniformBlocks[0].slot = 0;
        desc.storageBufferCount = 2;
        desc.storageBuffers[0].name = "SrcData";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "DstData";
        desc.storageBuffers[1].slot = 1;
        desc.debugName = i == 0 ? "ocean fft" : "ocean fft last";
        (i == 0 ? gpu.fft : gpu.fftLast) = driver->createComputePipeline(desc);
    }
    {
        prisma::ComputePipelineDesc desc;
        desc.shader = gpu.shaders[3];
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "OceanParams";
        desc.uniformBlocks[0].slot = 0;
        desc.storageBufferCount = 1;
        desc.storageBuffers[0].name = "InputDxyz";
        desc.storageBuffers[0].slot = 0;
        desc.storageTextureCount = 1;
        desc.storageTextures[0].name = "output_image";
        desc.storageTextures[0].slot = 0;
        desc.debugName = "ocean displacement";
        gpu.displacement = driver->createComputePipeline(desc);
    }
    {
        prisma::ComputePipelineDesc desc;
        desc.shader = gpu.shaders[4];
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "OceanParams";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 1;
        desc.textures[0].name = "texture_displacementmap";
        desc.textures[0].slot = 0;
        desc.storageTextureCount = 1;
        desc.storageTextures[0].name = "output_image";
        desc.storageTextures[0].slot = 0;
        desc.debugName = "ocean gradient";
        gpu.gradient = driver->createComputePipeline(desc);
    }

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::None;
    samplerDesc.addressU = prisma::AddressMode::Repeat;
    samplerDesc.addressV = prisma::AddressMode::Repeat;
    samplerDesc.addressW = prisma::AddressMode::Repeat;
    samplerDesc.debugName = "ocean displacement";
    gpu.sampler = driver->createSampler(samplerDesc);

    ct::Vector<Math::Vec2> h0;
    ct::Vector<float> omega;
    initialSpectrum(params, h0, omega);

    const std::uint32_t fieldBytes = kFftSlices * kFieldCount * 2u * sizeof(float);
    ct::Vector<float> zeros;
    zeros.resize(kFftSlices * kFieldCount * 2u, 0.0f);

    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Storage;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = kInputCount * 2u * sizeof(float);
    bufferDesc.data = h0.data();
    bufferDesc.debugName = "ocean h0";
    gpu.h0 = driver->createBuffer(bufferDesc);
    bufferDesc.size = kInputCount * sizeof(float);
    bufferDesc.data = omega.data();
    bufferDesc.debugName = "ocean omega";
    gpu.omega = driver->createBuffer(bufferDesc);
    bufferDesc.size = fieldBytes;
    bufferDesc.data = zeros.data();
    bufferDesc.debugName = "ocean ht";
    gpu.ht = driver->createBuffer(bufferDesc);
    bufferDesc.debugName = "ocean dxyz";
    gpu.dxyz = driver->createBuffer(bufferDesc);
    bufferDesc.debugName = "ocean fft temp";
    gpu.temp = driver->createBuffer(bufferDesc);

    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.size = sizeof(OceanParamsGpu);
    bufferDesc.data = nullptr;
    bufferDesc.debugName = "ocean params";
    gpu.params = driver->createBuffer(bufferDesc);

    gpu.fftStride = alignUp(static_cast<std::uint32_t>(sizeof(FftParamsGpu)),
            caps.uniformBufferOffsetAlignment > 0 ? caps.uniformBufferOffsetAlignment : 1u);
    ct::Vector<std::uint8_t> plan;
    plan.resize(static_cast<std::size_t>(gpu.fftStride) * kFftPasses, 0);
    {
        const std::uint32_t threadCount = kFftSlices * kFieldCount / 8u;
        std::uint32_t ostride = kFieldCount / 8u;
        std::uint32_t istride = ostride;
        std::uint32_t pstride = kSize;
        double phaseBase = -kTwoPi / (static_cast<double>(kSize) * static_cast<double>(kSize));
        for (std::uint32_t pass = 0; pass < kFftPasses; ++pass)
        {
            if (pass > 0)
            {
                istride /= 8u;
                phaseBase *= 8.0;
            }
            if (pass == 3)
            {
                ostride /= kSize;
                pstride = 1;
            }
            FftParamsGpu block;
            block.strides[0] = threadCount;
            block.strides[1] = ostride;
            block.strides[2] = istride;
            block.strides[3] = pstride;
            block.phase[0] = static_cast<float>(phaseBase);
            block.phase[1] = block.phase[2] = block.phase[3] = 0.0f;
            memcpy(plan.data() + static_cast<std::size_t>(gpu.fftStride) * pass, &block,
                    sizeof(block));
        }
    }
    bufferDesc.size = static_cast<std::uint32_t>(plan.size());
    bufferDesc.data = plan.data();
    bufferDesc.debugName = "ocean fft plan";
    gpu.fftParams = driver->createBuffer(bufferDesc);

    prisma::TextureDesc textureDesc;
    textureDesc.format = prisma::TextureFormat::RGBA16F;
    textureDesc.width = kSize;
    textureDesc.height = kSize;
    textureDesc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
    textureDesc.debugName = "ocean displacement";
    gpu.displacementMap = mTextures.create(textureDesc);
    std::uint32_t mips = 1;
    for (std::uint32_t size = kSize; size > 1; size >>= 1) ++mips;
    textureDesc.mipLevels = mips;
    textureDesc.debugName = "ocean gradient";
    gpu.gradientMap = mTextures.create(textureDesc);

    {
        prisma::TextureDesc perlinDesc;
        perlinDesc.format = prisma::TextureFormat::RGBA16F;
        perlinDesc.width = 64;
        perlinDesc.height = 64;
        perlinDesc.mipLevels = 7;
        perlinDesc.data = perlin;
        perlinDesc.debugName = "ocean perlin";
        gpu.perlinMap = mTextures.create(perlinDesc);
        if (mTextures.valid(gpu.perlinMap))
        {
            const unsigned char* source = perlin;
            for (std::uint32_t mip = 0; mip < perlinDesc.mipLevels; ++mip)
            {
                const std::uint32_t size = 64u >> mip;
                if (mip > 0) driver->updateTexture(mTextures.gpu(gpu.perlinMap), mip, 0, source);
                source += size * size * 8u;
            }
        }
    }

    const bool ok = gpu.simulate.valid() && gpu.fft.valid() && gpu.fftLast.valid() &&
                    gpu.displacement.valid() && gpu.gradient.valid() && gpu.sampler.valid() &&
                    gpu.h0.valid() && gpu.omega.valid() && gpu.ht.valid() && gpu.dxyz.valid() &&
                    gpu.temp.valid() && gpu.params.valid() && gpu.fftParams.valid() &&
                    mTextures.valid(gpu.displacementMap) && mTextures.valid(gpu.gradientMap) &&
                    mTextures.valid(gpu.perlinMap);
    if (!ok)
    {
        log_error("ocean: could not create the simulation resources");
        release();
        return false;
    }
    return true;
}

void OceanSimulation::release()
{
    if (!mGpu) return;
    Gpu& gpu = *mGpu;
    prisma::Driver* driver = gpu.driver;
    if (gpu.simulate.valid()) driver->destroy(gpu.simulate);
    if (gpu.fft.valid()) driver->destroy(gpu.fft);
    if (gpu.fftLast.valid()) driver->destroy(gpu.fftLast);
    if (gpu.displacement.valid()) driver->destroy(gpu.displacement);
    if (gpu.gradient.valid()) driver->destroy(gpu.gradient);
    for (int i = 0; i < 5; ++i)
        if (gpu.shaders[i].valid()) driver->destroy(gpu.shaders[i]);
    if (gpu.sampler.valid()) driver->destroy(gpu.sampler);
    prisma::BufferHandle* buffers[7] = { &gpu.h0, &gpu.omega, &gpu.ht, &gpu.dxyz, &gpu.temp,
        &gpu.params, &gpu.fftParams };
    for (int i = 0; i < 7; ++i)
        if (buffers[i]->valid()) driver->destroy(*buffers[i]);
    if (gpu.displacementMap.valid()) mTextures.destroy(gpu.displacementMap);
    if (gpu.gradientMap.valid()) mTextures.destroy(gpu.gradientMap);
    if (gpu.perlinMap.valid()) mTextures.destroy(gpu.perlinMap);
    delete mGpu;
    mGpu = nullptr;
}

bool OceanSimulation::valid() const
{
    return mGpu != nullptr;
}

const OceanSimulationParams& OceanSimulation::params() const
{
    return mParams;
}

float OceanSimulation::time() const
{
    return mTime;
}

TextureId OceanSimulation::displacementMap() const
{
    return mGpu ? mGpu->displacementMap : TextureId();
}

TextureId OceanSimulation::gradientMap() const
{
    return mGpu ? mGpu->gradientMap : TextureId();
}

TextureId OceanSimulation::perlinMap() const
{
    return mGpu ? mGpu->perlinMap : TextureId();
}

void OceanSimulation::update(float time)
{
    if (!mGpu) return;
    Gpu& gpu = *mGpu;
    prisma::Driver* driver = gpu.driver;
    mTime = time;

    OceanParamsGpu block;
    block.dims[0] = kSize;
    block.dims[1] = kInputWidth;
    block.dims[2] = kSize;
    block.dims[3] = kSize;
    block.offsets[0] = kFieldCount;
    block.offsets[1] = block.offsets[2] = block.offsets[3] = 0;
    block.values[0] = time;
    block.values[1] = mParams.timeScale;
    block.values[2] = mParams.choppyScale;
    block.values[3] = static_cast<float>(kSize) / mParams.patchLength;
    driver->updateBuffer(gpu.params, 0, &block, sizeof(block));

    const std::uint32_t fieldBytes = kFftSlices * kFieldCount * 2u * sizeof(float);
    const std::uint32_t groups = (kSize + kTileSize - 1) / kTileSize;
    const prisma::TextureHandle displacement = mTextures.gpu(gpu.displacementMap);
    const prisma::TextureHandle gradient = mTextures.gpu(gpu.gradientMap);

    driver->beginComputePass();

    driver->bindPipeline(gpu.simulate);
    driver->bindUniformBuffer(0, gpu.params, 0, sizeof(block));
    driver->bindStorageBuffer(0, gpu.h0, 0, kInputCount * 2u * sizeof(float));
    driver->bindStorageBuffer(1, gpu.omega, 0, kInputCount * sizeof(float));
    driver->bindStorageBuffer(2, gpu.ht, 0, fieldBytes);
    driver->dispatch(groups, groups, 1);

    const std::uint32_t fftGroups = kFftSlices * kFieldCount / 8u / kFftGranularity;
    const prisma::BufferHandle sources[kFftPasses] = { gpu.ht, gpu.temp, gpu.dxyz, gpu.temp,
        gpu.dxyz, gpu.temp };
    for (std::uint32_t pass = 0; pass < kFftPasses; ++pass)
    {
        driver->bindPipeline(pass + 1 == kFftPasses ? gpu.fftLast : gpu.fft);
        driver->bindUniformBuffer(0, gpu.fftParams, gpu.fftStride * pass, sizeof(FftParamsGpu));
        driver->bindStorageBuffer(0, sources[pass], 0, fieldBytes);
        driver->bindStorageBuffer(1, (pass & 1u) ? gpu.dxyz : gpu.temp, 0, fieldBytes);
        driver->dispatch(fftGroups, 1, 1);
    }

    driver->bindPipeline(gpu.displacement);
    driver->bindUniformBuffer(0, gpu.params, 0, sizeof(block));
    driver->bindStorageBuffer(0, gpu.dxyz, 0, fieldBytes);
    driver->bindStorageTexture(0, displacement, 0, prisma::StorageAccess::Write);
    driver->dispatch(groups, groups, 1);

    driver->bindPipeline(gpu.gradient);
    driver->bindUniformBuffer(0, gpu.params, 0, sizeof(block));
    driver->bindTexture(0, displacement, gpu.sampler);
    driver->bindStorageTexture(0, gradient, 0, prisma::StorageAccess::Write);
    driver->dispatch(groups, groups, 1);

    driver->endComputePass();
    driver->generateMipmaps(gradient);
}

bool OceanSimulation::readFields(ct::Vector<float>& out)
{
    if (!mGpu) return false;
    out.resize(kFftSlices * kFieldCount * 2u);
    return mGpu->driver->readBuffer(mGpu->dxyz, 0,
            static_cast<std::uint32_t>(out.size() * sizeof(float)), out.data());
}

} // namespace prisma3d
