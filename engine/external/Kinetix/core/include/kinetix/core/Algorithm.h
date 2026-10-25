#pragma once

namespace kinetix {

template <typename T>
constexpr T Min(T a, T b) { return a < b ? a : b; }

template <typename T>
constexpr T Max(T a, T b) { return a > b ? a : b; }

template <typename T>
constexpr T Clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

template <typename T>
void Swap(T& a, T& b) { T tmp = a; a = b; b = tmp; }

template <typename T>
constexpr T&& Move(T& t) { return static_cast<T&&>(t); }

template <typename It, typename T>
It Find(It first, It last, const T& value)
{
    for (; first != last; ++first)
        if (*first == value)
            return first;
    return last;
}

template <typename It, typename Pred>
It FindIf(It first, It last, Pred pred)
{
    for (; first != last; ++first)
        if (pred(*first))
            return first;
    return last;
}

template <typename It, typename T>
It LowerBound(It first, It last, const T& value)
{
    while (first != last)
    {
        It mid = first + (last - first) / 2;
        if (*mid < value)
            first = mid + 1;
        else
            last = mid;
    }
    return first;
}

template <typename It, typename T, typename Cmp>
It LowerBound(It first, It last, const T& value, Cmp cmp)
{
    while (first != last)
    {
        It mid = first + (last - first) / 2;
        if (cmp(*mid, value))
            first = mid + 1;
        else
            last = mid;
    }
    return first;
}

template <typename It, typename T>
It UpperBound(It first, It last, const T& value)
{
    while (first != last)
    {
        It mid = first + (last - first) / 2;
        if (!(value < *mid))
            first = mid + 1;
        else
            last = mid;
    }
    return first;
}

template <typename It, typename T, typename Cmp>
It UpperBound(It first, It last, const T& value, Cmp cmp)
{
    while (first != last)
    {
        It mid = first + (last - first) / 2;
        if (!cmp(value, *mid))
            first = mid + 1;
        else
            last = mid;
    }
    return first;
}

template <typename It, typename T>
bool BinarySearch(It first, It last, const T& value)
{
    It it = LowerBound(first, last, value);
    return it != last && !(value < *it);
}

template <typename It, typename T>
void Fill(It first, It last, const T& value)
{
    for (; first != last; ++first)
        *first = value;
}

template <typename It>
It Unique(It first, It last)
{
    if (first == last)
        return last;
    It result = first;
    while (++first != last)
    {
        if (!(*result == *first) && ++result != first)
            *result = *first;
    }
    return ++result;
}

template <typename It, typename Pred>
It RemoveIf(It first, It last, Pred pred)
{
    first = FindIf(first, last, pred);
    if (first == last)
        return last;
    It it = first;
    while (++it != last)
    {
        if (!pred(*it))
            *first++ = *it;
    }
    return first;
}

} // namespace kinetix
