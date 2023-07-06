#ifndef BAXTER_BRIDGE_CONVERSIONS_H
#define BAXTER_BRIDGE_CONVERSIONS_H

#include <vector>
#include <array>
#include <algorithm>

namespace baxter_bridge
{

// basic types + their vectors if same types
template <class Src, class Dst>
inline void convert(const Src &src, Dst &dst)
{
  dst = src;
}

// vectors of different types
template <class Src, class Dst>
std::enable_if_t<!std::is_same_v<Src,Dst>, void>
convert(const std::vector<Src>& src, std::vector<Dst> &dst)
{
  dst.clear();
  dst.reserve(src.size());
  Dst converted;
  std::transform(src.begin(), src.end(), std::back_inserter(dst),
                 [&converted](const auto &elem)
  {convert(elem,converted);return converted;});
}

// arrays are std in ROS 2 vs boost in ROS 1
template <class Src, class DstBoostArray, size_t N>
inline void convert(const std::array<Src, N> &src, DstBoostArray &dst)
{
  for(size_t i = 0; i < N; ++i)
    convert(src[i], dst[i]);
}

template <class SrcBoostArray, class Dst, size_t N>
inline void convert(const SrcBoostArray &src, std::array<Dst,N> &dst)
{
  for(size_t i = 0; i < N; ++i)
    convert(src[i], dst[i]);
}

}

#endif // CONVERSIONS_H
