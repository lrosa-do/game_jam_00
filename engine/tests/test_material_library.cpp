#include "Check.h"
#include "MaterialFields.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/render/MaterialLibrary.h"

#include "platform.h"

#include <string.h>

using namespace prisma3d;

namespace
{

ct::String fixture(const char* name)
{
    return ct::String(PRISMA3D_FIXTURES_DIR) + "/materials/" + name;
}

bool parseText(const char* text, MaterialLibraryData& data, ct::String& error)
{
    return MaterialLibraryFormat::parse(text, strlen(text), data, error);
}

bool loadFixture(const char* name, MaterialLibraryData& data, ct::String& error)
{
    return MaterialLibraryFormat::load(fixture(name), data, error);
}

ct::String readFixtureText(const char* name)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(fixture(name));
    return ct::String(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

void expectError(const char* text, const char* expected)
{
    MaterialLibraryData data;
    ct::String error;
    const bool ok = parseText(text, data, error);
    if (ok || !(error == expected))
        printf("expected error '%s' for:\n%s\ngot '%s'\n", expected, text, error.c_str());
    CHECK(!ok);
    CHECK(error == expected);
    CHECK(data.materials.empty());
}

void testEveryKey()
{
    MaterialLibraryData data;
    ct::String error;
    CHECK(loadFixture("all_keys.mat", data, error));
    CHECK(error.empty());
    CHECK(data.warnings.empty());
    CHECK(data.materials.size() == 2);
    if (data.materials.size() != 2) return;

    const MaterialDef& everything = data.materials[0];
    const MaterialInfo expected = everythingMaterial();
    CHECK(everything.info.name == "everything");
    CHECK(materialFieldDifferences(everything.info, expected) == 0);
    CHECK(materialFieldDifferences(everything.info, MaterialInfo()) == kMaterialFieldCount);

    const char* const paths[kMaterialTextureCount] = { "tex/albedo.png", "tex/metal.png",
        "tex/rough.png", "tex/emission.png", "tex/normal.png", "tex/ao.png", "tex/coat.png",
        "tex/orm.png", "tex/detail_albedo.png", "tex/detail_normal.png", "tex/detail_mask.png",
        "tex/refraction.png" };
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(everything.paths[slot] == paths[slot]);

    const MaterialDef& plain = data.materials[1];
    CHECK(plain.info.name == "plain");
    CHECK(plain.info.albedo.x == 0.75f && plain.info.albedo.w == 1.0f);
    CHECK(plain.info.roughness == 0.9f);
    CHECK(materialFieldDifferences(plain.info, MaterialInfo()) == 2);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(plain.paths[slot].empty());

    CHECK(data.find("plain") == &data.materials[1]);
    CHECK(data.find("missing") == nullptr);
    CHECK(data.find(nullptr) == nullptr);

    ct::String text;
    MaterialLibraryFormat::write(data, text);
    CHECK(text == readFixtureText("all_keys.mat"));

    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::Albedo), "albedo") == 0);
    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::Orm), "orm") == 0);
    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::Count), "") == 0);
}

void testSyntax()
{
    MaterialLibraryData data;
    ct::String error;
    CHECK(loadFixture("syntax.mat", data, error));
    CHECK(data.warnings.empty());
    CHECK(data.materials.size() == 4);
    if (data.materials.size() != 4) return;

    const MaterialInfo& scalars = data.materials[0].info;
    CHECK(scalars.name == "scalars");
    CHECK(scalars.roughness == 1.0f);
    CHECK(scalars.metallic == 0.5f);
    CHECK(scalars.specular == -2.5f);
    CHECK(scalars.albedo.x == 1.0f && scalars.albedo.y == 0.0f && scalars.albedo.w == 1.0f);
    CHECK(scalars.emission.x == 0.5f && scalars.emission.y == 0.25f && scalars.emission.z == 0.0f);

    const MaterialInfo& forms = data.materials[1].info;
    CHECK(forms.cull == MaterialCull::Disabled);
    CHECK(forms.shading == MaterialShading::Lit);
    CHECK(forms.depthTest);
    CHECK(forms.alphaScissorThreshold == 0.1f);

    CHECK(data.materials[2].info.name == "dup" && data.materials[3].info.name == "dup");
    CHECK(data.materials[2].info.roughness == 0.1f && data.materials[3].info.roughness == 0.2f);
    CHECK(data.find("dup") == &data.materials[2]);

    MaterialLibraryData empty;
    CHECK(parseText("", empty, error) && empty.materials.empty());
    CHECK(parseText("  // only a comment\n# and another\n", empty, error));
    CHECK(parseText("material \"m\" { }", empty, error) && empty.materials.size() == 1);
    CHECK(materialFieldDifferences(empty.materials[0].info, MaterialInfo()) == 0);
}

void testUnknownKeys()
{
    MaterialLibraryData data;
    ct::String error;
    CHECK(loadFixture("unknown.mat", data, error));
    CHECK(data.materials.size() == 1);
    CHECK(data.warnings.size() == 9);
    const char* const expected[] = { "line 2: unknown key 'height' ignored",
        "line 3: unknown key 'height_scale' ignored", "line 4: unknown key 'reflection' ignored",
        "line 5: unknown key 'uv_scroll' ignored", "line 6: unknown key 'queue' ignored",
        "line 8: unknown block 'param' ignored", "line 12: unknown key 'albedo_frames' ignored",
        "line 13: unknown key 'cast_shadow' ignored", "line 14: unknown key 'shader' ignored" };
    for (std::size_t i = 0; i < 9 && i < data.warnings.size(); ++i)
    {
        if (!(data.warnings[i] == expected[i]))
            printf("warning %zu: '%s'\n", i, data.warnings[i].c_str());
        CHECK(data.warnings[i] == expected[i]);
    }
    if (data.materials.empty()) return;
    const MaterialInfo& info = data.materials[0].info;
    CHECK(info.roughness == 0.3f);
    CHECK(info.metallic == 1.0f);
    CHECK(materialFieldDifferences(info, MaterialInfo()) == 2);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        CHECK(data.materials[0].paths[slot].empty());
}

void testErrors()
{
    expectError("material \"m\" { ", "line 1: expected key, got end of file");
    expectError("material \"m", "line 1: unterminated string");
    expectError("material \"m\" {\n  roughness = \"a\n}", "line 2: unterminated string");
    expectError("material { }", "line 1: expected material name string, got '{'");
    expectError("material \"\" { }", "line 1: material name must not be empty");
    expectError("material \"m\" { color = 1.2.3 }", "line 1: malformed number");
    expectError("material \"m\" { color = - }", "line 1: unexpected character '-'");
    expectError("material \"m\" { metallic Opaque }",
            "line 1: expected '=', got identifier 'Opaque'");
    expectError("\n\nmaterial \"m\" {\n\n    cull = Bogus\n}",
            "line 5: 'cull': unknown value 'Bogus'");
    expectError("material \"m\" { cull = \"Back\" }",
            "line 1: 'cull' expects a name without quotes");
    expectError("material \"m\" { albedo = ground }",
            "line 1: 'albedo' expects a quoted path or a list of quoted paths");
    expectError("material \"m\" { lit = 1 }", "line 1: 'lit' expects true or false");
    expectError("material \"m\" { roughness = true }", "line 1: 'roughness' expects a number");
    expectError("material \"m\" { color = (1, 0) }",
            "line 1: 'color' expects (r, g, b) or (r, g, b, a)");
    expectError("material \"m\" { emission_color = (1, 0, 0, 1) }",
            "line 1: 'emission_color' expects (x, y, z)");
    expectError("material \"m\" { uv1_scale = 2 }", "line 1: 'uv1_scale' expects (x, y, z)");
    expectError("material \"m\" { color = (1, 2, 3, 4, 5) }",
            "line 1: a vector has at most four components");
    expectError("material \"m\" { color = (1) }", "line 1: a vector has at least two components");
    expectError("material \"m\" { color = (1, , 2) }", "line 1: expected number, got ','");
    expectError("material \"m\" { metallic_channel = Purple }",
            "line 1: 'metallic_channel': unknown value 'Purple'");
    expectError("material \"m\" { blend = Mix }", "line 1: 'blend': unknown value 'Mix'");
    expectError("material \"m\" { albedo = 5 }",
            "line 1: 'albedo' expects a quoted path or a list of quoted paths");
    expectError("material \"m\" { thing { a = 1 ", "line 1: block 'thing' is never closed");
    expectError("material \"m\" { thing = [1] }",
            "line 1: expected a quoted string inside [ ], got number");
    expectError("material \"a\" { }\n\nfoo", "line 3: expected 'material', got identifier 'foo'");
    expectError("material \"m\" : \"parent\" { }",
            "line 1: unexpected character ':' (a material does not inherit from another; every "
            "key it needs is written in its own block)");
    expectError("material \"m\" { roughness = 1 } }", "line 1: expected 'material', got '}'");
    expectError("material \"m\" { color = 1.5abc }", "line 1: malformed number");
    expectError("material \"a\" { }\nmaterial \"b\" {\n    roughness = 0.5\n    metallic\n}",
            "line 5: expected '=', got '}'");
}

void testSequences()
{
    const std::size_t albedo = static_cast<std::size_t>(MaterialTexture::Albedo);
    const std::size_t normal = static_cast<std::size_t>(MaterialTexture::Normal);
    const std::size_t emission = static_cast<std::size_t>(MaterialTexture::Emission);

    const char* const source = "material \"water\" {\n"
                               "    albedo = [\"w/0.png\", \"w/1.png\", \"w/2.png\"]\n"
                               "    albedo_fps = 12\n"
                               "    normal = \"n.png\"\n"
                               "    emission = [\"e0.png\"]\n"
                               "}\n";
    MaterialLibraryData data;
    ct::String error;
    CHECK(parseText(source, data, error));
    CHECK(error.empty());
    CHECK(data.warnings.empty());
    CHECK(data.materials.size() == 1);
    if (data.materials.size() != 1) return;
    const MaterialDef& water = data.materials[0];
    CHECK(water.paths[albedo].empty());
    CHECK(water.sequences[albedo].size() == 3);
    CHECK(water.sequences[albedo].size() == 3 && water.sequences[albedo][0] == "w/0.png" &&
            water.sequences[albedo][1] == "w/1.png" && water.sequences[albedo][2] == "w/2.png");
    CHECK(water.sequenceFps[albedo] == 12.0f);
    CHECK(water.paths[normal] == "n.png" && water.sequences[normal].empty());
    CHECK(water.sequenceFps[normal] == 0.0f);
    CHECK(water.paths[emission].empty() && water.sequences[emission].size() == 1);
    CHECK(water.sequences[emission].size() == 1 && water.sequences[emission][0] == "e0.png");
    CHECK(water.sequenceFps[emission] == 0.0f);
    CHECK(materialFieldDifferences(water.info, MaterialInfo()) == 0);

    ct::String text;
    MaterialLibraryFormat::write(data, text);
    const char* const expected = "material \"water\" {\n"
                                 "    albedo           = [\"w/0.png\", \"w/1.png\", \"w/2.png\"]\n"
                                 "    albedo_fps       = 12\n"
                                 "    emission         = [\"e0.png\"]\n"
                                 "    normal           = \"n.png\"\n"
                                 "}\n";
    if (!(text == expected)) printf("sequence written as:\n%s", text.c_str());
    CHECK(text == expected);

    MaterialLibraryData again;
    CHECK(parseText(text.c_str(), again, error));
    CHECK(again.materials.size() == 1);
    if (again.materials.size() == 1) CHECK(materialDefDifferences(again.materials[0], water) == 0);
    ct::String twice;
    MaterialLibraryFormat::write(again, twice);
    CHECK(twice == text);

    MaterialLibraryData last;
    CHECK(parseText("material \"m\" { albedo = \"x.png\" albedo = [\"y.png\"] normal = [\"a.png\"] "
                    "normal = \"b.png\" }",
            last, error));
    CHECK(last.materials.size() == 1);
    if (last.materials.size() == 1)
    {
        CHECK(last.materials[0].paths[albedo].empty() &&
                last.materials[0].sequences[albedo].size() == 1);
        CHECK(last.materials[0].paths[normal] == "b.png" &&
                last.materials[0].sequences[normal].empty());
    }

    MaterialLibraryData rate;
    CHECK(parseText("material \"m\" {\n    albedo = \"a.png\"\n    albedo_fps = 5\n    "
                    "orm_fps = 2\n}\n",
            rate, error));
    CHECK(rate.warnings.size() == 2);
    if (rate.warnings.size() == 2)
    {
        CHECK(rate.warnings[0] == "line 3: 'albedo_fps' ignored: 'albedo' is not a sequence");
        CHECK(rate.warnings[1] == "line 4: 'orm_fps' ignored: 'orm' is not a sequence");
    }
    ct::String plain;
    MaterialLibraryFormat::write(rate, plain);
    CHECK(plain == "material \"m\" {\n    albedo           = \"a.png\"\n}\n");

    MaterialLibraryData rateFirst;
    CHECK(parseText("material \"m\" { albedo_fps = 4 albedo = [\"a.png\", \"b.png\"] }", rateFirst,
            error));
    CHECK(rateFirst.warnings.empty());
    CHECK(rateFirst.materials.size() == 1 && rateFirst.materials[0].sequenceFps[albedo] == 4.0f);

    expectError("material \"m\" {\n    albedo = []\n}",
            "line 2: 'albedo': a sequence needs at least one file");
    expectError("material \"m\" { albedo = [1, 2] }",
            "line 1: expected a quoted string inside [ ], got number");
    expectError("material \"m\" { albedo = [\"a.png\" }", "line 1: expected ']', got '}'");
    expectError("material \"m\" { albedo = [\"a.png\" \"b.png\"] }",
            "line 1: expected ']', got string 'b.png'");
    expectError("material \"m\" {\n\n    albedo = [\"a.png\"]\n    albedo_fps = 0\n}",
            "line 4: 'albedo_fps' expects a number greater than 0");
    expectError("material \"m\" { albedo_fps = -3 }",
            "line 1: 'albedo_fps' expects a number greater than 0");
    expectError("material \"m\" { albedo_fps = \"12\" }",
            "line 1: 'albedo_fps' expects a number greater than 0");
    expectError("material \"m\" { albedo_fps = (1, 2) }",
            "line 1: 'albedo_fps' expects a number greater than 0");
    expectError("material \"m\" { albedo = true }",
            "line 1: 'albedo' expects a quoted path or a list of quoted paths");

    ct::String many("material \"m\" {\n    albedo = [");
    for (std::size_t i = 0; i < kMaxTextureSequence; ++i)
    {
        if (i > 0) many += ", ";
        many += "\"f.png\"";
    }
    many += "]\n}\n";
    MaterialLibraryData full;
    CHECK(parseText(many.c_str(), full, error));
    CHECK(full.materials.size() == 1 && full.materials[0].sequences[albedo].size() == 256);
    ct::String tooMany("material \"m\" {\n    albedo = [");
    for (std::size_t i = 0; i < kMaxTextureSequence + 1; ++i)
    {
        if (i > 0) tooMany += ", ";
        tooMany += "\"f.png\"";
    }
    tooMany += "]\n}\n";
    expectError(tooMany.c_str(), "line 2: 'albedo': a sequence has at most 256 files");
}

void testDetailKeys()
{
    MaterialLibraryData data;
    ct::String error;
    CHECK(parseText("material \"d\" {\n"
                    "    detail = true\n"
                    "    detail_albedo = \"a.png\"\n"
                    "    detail_normal = [\"n0.png\", \"n1.png\"]\n"
                    "    detail_normal_fps = 6\n"
                    "    detail_mask = \"m.png\"\n"
                    "    detail_blend = Subtract\n"
                    "    detail_uv = Uv2\n"
                    "    uv2_scale = (4, 5, 1)\n"
                    "    uv2_offset = (0.5, 0.25, 0)\n"
                    "}\n",
            data, error));
    CHECK(data.warnings.empty());
    CHECK(data.materials.size() == 1);
    if (data.materials.size() != 1) return;
    const MaterialDef& def = data.materials[0];
    CHECK(def.info.detail);
    CHECK(def.info.detailBlend == MaterialBlend::Sub);
    CHECK(def.info.detailUv == MaterialDetailUv::Uv2);
    CHECK(def.info.uv2Scale.x == 4.0f && def.info.uv2Scale.y == 5.0f &&
            def.info.uv2Scale.z == 1.0f);
    CHECK(def.info.uv2Offset.x == 0.5f && def.info.uv2Offset.y == 0.25f);
    CHECK(materialFieldDifferences(def.info, MaterialInfo()) == 5);
    const std::size_t albedo = static_cast<std::size_t>(MaterialTexture::DetailAlbedo);
    const std::size_t normal = static_cast<std::size_t>(MaterialTexture::DetailNormal);
    const std::size_t mask = static_cast<std::size_t>(MaterialTexture::DetailMask);
    CHECK(def.paths[albedo] == "a.png" && def.paths[mask] == "m.png");
    CHECK(def.sequences[normal].size() == 2 && def.sequenceFps[normal] == 6.0f);

    ct::String text;
    MaterialLibraryFormat::write(data, text);
    const char* const expected = "material \"d\" {\n"
                                 "    uv2_scale        = (4, 5, 1)\n"
                                 "    uv2_offset       = (0.5, 0.25, 0)\n"
                                 "    detail           = true\n"
                                 "    detail_albedo    = \"a.png\"\n"
                                 "    detail_normal    = [\"n0.png\", \"n1.png\"]\n"
                                 "    detail_normal_fps = 6\n"
                                 "    detail_mask      = \"m.png\"\n"
                                 "    detail_blend     = Subtract\n"
                                 "    detail_uv        = Uv2\n"
                                 "}\n";
    if (!(text == expected)) printf("detail written as:\n%s", text.c_str());
    CHECK(text == expected);
    MaterialLibraryData again;
    CHECK(parseText(text.c_str(), again, error));
    CHECK(again.materials.size() == 1);
    if (again.materials.size() == 1) CHECK(materialDefDifferences(again.materials[0], def) == 0);

    const char* const modes[] = { "Alpha", "Add", "Multiply", "Subtract" };
    const MaterialBlend blends[] = { MaterialBlend::Mix, MaterialBlend::Add, MaterialBlend::Mul,
        MaterialBlend::Sub };
    for (int i = 0; i < 4; ++i)
    {
        ct::String one("material \"m\" { detail_blend = ");
        one += modes[i];
        one += " }";
        MaterialLibraryData parsed;
        CHECK(parseText(one.c_str(), parsed, error));
        CHECK(parsed.materials.size() == 1 && parsed.materials[0].info.detailBlend == blends[i]);
        ct::String written;
        MaterialLibraryFormat::write(parsed, written);
        CHECK((i == 0) == (written == "material \"m\" {\n}\n"));
    }
    MaterialLibraryData uv1;
    CHECK(parseText("material \"m\" { detail_uv = Uv1 detail = false }", uv1, error));
    ct::String quiet;
    MaterialLibraryFormat::write(uv1, quiet);
    CHECK(quiet == "material \"m\" {\n}\n");

    expectError("material \"m\" { detail = 1 }", "line 1: 'detail' expects true or false");
    expectError("material \"m\" { detail_blend = Screen }",
            "line 1: 'detail_blend': unknown value 'Screen'");
    expectError("material \"m\" { detail_blend = Mix }",
            "line 1: 'detail_blend': unknown value 'Mix'");
    expectError("material \"m\" { detail_blend = \"Add\" }",
            "line 1: 'detail_blend' expects a name without quotes");
    expectError("material \"m\" { detail_uv = Uv3 }", "line 1: 'detail_uv': unknown value 'Uv3'");
    expectError("material \"m\" { uv2_scale = 2 }", "line 1: 'uv2_scale' expects (x, y, z)");
    expectError("material \"m\" {\n    uv2_offset = (1, 2)\n}",
            "line 2: 'uv2_offset' expects (x, y, z)");
    expectError("material \"m\" { detail_mask = 3 }",
            "line 1: 'detail_mask' expects a quoted path or a list of quoted paths");
    expectError("material \"m\" { detail_albedo = [] }",
            "line 1: 'detail_albedo': a sequence needs at least one file");
    expectError("material \"m\" { detail_albedo_fps = 0 }",
            "line 1: 'detail_albedo_fps' expects a number greater than 0");

    MaterialLibraryData rate;
    CHECK(parseText("material \"m\" {\n    detail_mask = \"m.png\"\n    detail_mask_fps = 3\n}\n",
            rate, error));
    CHECK(rate.warnings.size() == 1);
    if (rate.warnings.size() == 1)
        CHECK(rate.warnings[0] ==
                "line 3: 'detail_mask_fps' ignored: 'detail_mask' is not a sequence");

    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::DetailAlbedo),
                  "detail_albedo") == 0);
    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::DetailNormal),
                  "detail_normal") == 0);
    CHECK(strcmp(MaterialLibraryFormat::textureKey(MaterialTexture::DetailMask), "detail_mask") ==
            0);
}

void testForeignFormat()
{
    MaterialLibraryData data;
    ct::String error;
    CHECK(!loadFixture("foreign_format.mat", data, error));
    const ct::String expected = fixture("foreign_format.mat") +
                                ": line 2: expected material name string, got identifier "
                                "'Scene'";
    if (!(error == expected)) printf("foreign: '%s'\n", error.c_str());
    CHECK(error == expected);
    CHECK(data.materials.empty());

    CHECK(!MaterialLibraryFormat::load(fixture("missing.mat"), data, error));
    CHECK(error == fixture("missing.mat") + ": not found or empty");
}

void testWriter()
{
    MaterialLibraryData data;
    ct::String text;
    MaterialLibraryFormat::write(data, text);
    CHECK(text.empty());

    MaterialDef bare;
    bare.info.name = "bare";
    data.materials.push_back(bare);
    MaterialLibraryFormat::write(data, text);
    CHECK(text == "material \"bare\" {\n}\n");

    MaterialDef second;
    second.info.name = "second";
    second.info.roughness = 0.25f;
    second.info.shading = MaterialShading::Unshaded;
    second.paths[static_cast<std::size_t>(MaterialTexture::Normal)] = "n.png";
    data.materials.push_back(second);
    text = ct::String();
    MaterialLibraryFormat::write(data, text);
    CHECK(text == "material \"bare\" {\n}\n\n"
                  "material \"second\" {\n"
                  "    lit              = false\n"
                  "    roughness        = 0.25\n"
                  "    normal           = \"n.png\"\n"
                  "}\n");
}

void testRealFilesRoundTrip()
{
    const char* const files[] = { "ruins_materials.mat", "cloister_materials.mat" };
    const std::size_t counts[] = { 7, 1 };
    for (std::size_t f = 0; f < 2; ++f)
    {
        MaterialLibraryData data;
        ct::String error;
        CHECK(loadFixture(files[f], data, error));
        CHECK(data.warnings.empty());
        CHECK(data.materials.size() == counts[f]);
        ct::String text;
        MaterialLibraryFormat::write(data, text);
        text += "\n";
        const ct::String original = readFixtureText(files[f]);
        if (!(text == original)) printf("%s does not write back identically\n", files[f]);
        CHECK(text == original);
    }

    MaterialLibraryData sponza;
    ct::String error;
    CHECK(loadFixture("sponza_materials.mat", sponza, error));
    CHECK(sponza.materials.size() == 26);
    CHECK(sponza.warnings.size() == 22);
    for (std::size_t i = 0; i < sponza.warnings.size(); ++i)
    {
        const ct::String& warning = sponza.warnings[i];
        const bool height = warning.find("unknown key 'height' ignored") != ct::String::npos;
        const bool scale = warning.find("unknown key 'height_scale' ignored") != ct::String::npos;
        CHECK(height || scale);
    }
    const MaterialDef* leaf = sponza.find("leaf");
    CHECK(leaf != nullptr);
    if (leaf)
    {
        CHECK(leaf->info.cull == MaterialCull::Disabled);
        CHECK(leaf->info.roughness == 0.9f);
        CHECK(leaf->paths[static_cast<std::size_t>(MaterialTexture::Albedo)] ==
                "models/sponza/textures/sponza_thorn_diff.png");
        CHECK(leaf->paths[static_cast<std::size_t>(MaterialTexture::Normal)] ==
                "models/sponza/textures/sponza_thorn_ddn.tga");
    }

    ct::String once;
    MaterialLibraryFormat::write(sponza, once);
    MaterialLibraryData again;
    CHECK(MaterialLibraryFormat::parse(once.c_str(), once.size(), again, error));
    CHECK(again.warnings.empty());
    CHECK(again.materials.size() == 26);
    ct::String twice;
    MaterialLibraryFormat::write(again, twice);
    CHECK(once == twice);
    for (std::size_t i = 0; i < again.materials.size() && i < sponza.materials.size(); ++i)
        CHECK(materialDefDifferences(again.materials[i], sponza.materials[i]) == 0);
}

void testSaveAndLoad()
{
    const ct::String path = ct::String(PRISMA3D_TEST_DIR) + "/material_library_saved.mat";
    MaterialLibraryData data;
    ct::String error;
    CHECK(loadFixture("all_keys.mat", data, error));
    CHECK(MaterialLibraryFormat::save(path, data, error));
    CHECK(error.empty());
    CHECK(!FileSystem::getSingleton().exists(path + ".tmp"));
    CHECK(MaterialLibraryFormat::save(path, data, error));

    MaterialLibraryData back;
    CHECK(MaterialLibraryFormat::load(path, back, error));
    CHECK(back.materials.size() == 2);
    ct::String first;
    ct::String second;
    MaterialLibraryFormat::write(data, first);
    MaterialLibraryFormat::write(back, second);
    CHECK(first == second);

    CHECK(!MaterialLibraryFormat::save(ct::String(PRISMA3D_TEST_DIR) + "/no_such_folder/x.mat",
            data, error));
    CHECK(!error.empty());
}

} // namespace

int main()
{
    testEveryKey();
    testSyntax();
    testUnknownKeys();
    testErrors();
    testSequences();
    testDetailKeys();
    testForeignFormat();
    testWriter();
    testRealFilesRoundTrip();
    testSaveAndLoad();
    return failures ? 1 : 0;
}
