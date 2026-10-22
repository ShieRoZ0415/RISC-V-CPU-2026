#!/usr/bin/env python3
"""Link the existing Verilator model to an AXI backpressure/stability harness."""
import re
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory
from regression import make_program

ROOT = Path(__file__).resolve().parents[1]
OBJ = ROOT / 'build/obj'
makefile = (OBJ / 'Vstudent_top.mk').read_text()
verilator_root = Path(re.search(r'^VERILATOR_ROOT\s*=\s*(.+)$', makefile, re.M)[1].strip())
subprocess.run(['g++', '-std=c++17', '-O2', '-I' + str(OBJ),
                '-I' + str(verilator_root / 'include'), str(ROOT / 'tests/axi_stress.cpp'),
                str(OBJ / 'Vstudent_top__ALL.a'), str(OBJ / 'verilated.o'),
                str(OBJ / 'verilated_vcd_c.o'), str(OBJ / 'verilated_threads.o'),
                '-pthread', '-latomic', '-o', str(ROOT / 'build/axi_stress')], check=True)
with TemporaryDirectory(prefix='rv32-axi-') as directory:
    path = Path(directory) / 'program.data'
    _, image = make_program()
    path.write_text(image)
    subprocess.run([str(ROOT / 'build/axi_stress'), str(path), '0'], check=True)
