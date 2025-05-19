
<img align="right" width="26%" src="./misc/logo.png">

Farmtrax
===

A C++ header-only for generating AB-lines for farm fields.

---

## Installation

### CMake

```cmake
FetchContent_Declare(
  farmtrax
  GIT_REPOSITORY https://github.com/bresilla/farmtrax.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(farmtrax)


target_link_libraries(<lib/bin> PRIVATE farmtrax::farmtrax)
```

---
