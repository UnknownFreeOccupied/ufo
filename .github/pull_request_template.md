# Description

<!--
Please include a summary of the change and which issue(s) are fixed. Please also include relevant motivation and context. List any dependencies that are required for this change.
-->

<!-- If this pull request fixes one or more issues, uncomment the line below
     and replace 123 with the relevant issue number(s), for example:
     "Fixes #123" or "Fixes #123, Fixes #456".
Fixes #123
-->

## Type of change

<!--
Please delete options that are not relevant.
-->

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] This change requires a documentation update

# How Has This Been Tested?

<!--
Please describe the tests that you ran to verify your changes. Provide instructions so we can reproduce. Please also list any relevant details for your test configuration.
-->

- [ ] Test A
- [ ] Test B

**Test Configuration**:
<!--
Please run `pixi run system_info` or `ufo --version` to get the system information. Copy the output here. It should look something like:
```text
UFO v1.0.0

================================= SYSTEM INFO ==================================
OS:                Ubuntu 24.04.4 LTS (Kernel: 6.17.0-14-generic)
CPU:               AMD Ryzen 5 3600X 6-Core Processor (6C/12T) @ x86_64
SIMD Extensions:   SSE2
RAM:               31.3 GB Total (21.9 GB)

GPU [Vulkan]:      NVIDIA GeForce GTX 1660 SUPER (Discrete GPU) [Driver:
                   590.48.01]
GPU [Vulkan]:      llvmpipe (LLVM 20.1.2, 256 bits) (CPU) [Driver: Mesa
                   25.2.8-0ubuntu0.24.04.1 (LLVM 20.1.2)]
GPU [OpenGL]:      NVIDIA GeForce GTX 1660 SUPER/PCIe/SSE2 [Driver: 3.3.0 NVIDIA
                   590.48.01]

Environment:       64-bit | Little Endian
Compiler:          GCC 14.2.0 | C++23 (202302)
Build:             Release [Feb 24 2026 23:25:10]
================================================================================
```
-->

# Checklist

- [ ] My code follows the style guidelines of this project
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes
- [ ] Any dependent changes have been merged and published in downstream modules
- [ ] I have added newly created `.cpp` and `.hpp` files to the `target_sources` in the `CMakeLists.txt` files
