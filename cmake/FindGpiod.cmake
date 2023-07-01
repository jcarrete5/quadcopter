find_package(PkgConfig REQUIRED)
pkg_search_module(Gpiod REQUIRED IMPORTED_TARGET libgpiodcxx>=1.6.2)
add_library(Gpiod::gpiod ALIAS PkgConfig::Gpiod)
