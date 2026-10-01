# Dependencies

CasADi and its IPOPT plugin, HSL MA97, and compiler runtime dependencies retain their own licenses. Install these separately; they are not included in this repository. See [CasADi](https://web.casadi.org/), [IPOPT](https://github.com/coin-or/Ipopt), and [HSL](https://licences.stfc.ac.uk/product/coin-hsl).

The protected guiding-search libraries implement this project's planar A* component. They are covered by the project noncommercial license. Their compiler support libraries, where statically linked, retain the applicable GCC Runtime Library Exception or LLVM runtime terms. No third-party optimization library is embedded in these search binaries.
