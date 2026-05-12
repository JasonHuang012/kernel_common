  #!/bin/bash
  # gen_cscope.sh - 生成 cscope 文件列表并建立索引

  find . -name "*.c" -o -name "*.h" -o -name "*.S" | \
      grep -v "^./arch/alpha" | \
      grep -v "^./arch/arc" | \
      grep -v "^./arch/csky" | \
      grep -v "^./arch/hexagon" | \
      grep -v "^./arch/loongarch" | \
      grep -v "^./arch/m68k" | \
      grep -v "^./arch/microblaze" | \
      grep -v "^./arch/mips" | \
      grep -v "^./arch/nios2" | \
      grep -v "^./arch/openrisc" | \
      grep -v "^./arch/parisc" | \
      grep -v "^./arch/powerpc" | \
      grep -v "^./arch/riscv" | \
      grep -v "^./arch/s390" | \
      grep -v "^./arch/sh" | \
      grep -v "^./arch/sparc" | \
      grep -v "^./arch/um" | \
      grep -v "^./arch/x86" | \
      grep -v "^./arch/xtensa" | \
      grep -v "^./Documentation" | \
      grep -v "^./tools" | \
      grep -v "^./samples" | \
      grep -v "^./\.git" | \
      grep -v "^./scripts" | \
      grep -v "^./rust" | \
      #grep -v "^./drivers" | \
      grep -v "^./sound" | \
      grep -v "^./crypto" | \
      grep -v "^./security" | \
      grep -v "^./net" \
      > cscope.files
