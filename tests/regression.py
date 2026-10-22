#!/usr/bin/env python3
"""Self-checking RV32IM programs; uses only Python and the built course simulator."""
import argparse
import random
import subprocess
import tempfile
from pathlib import Path

MASK = 0xffffffff


def signed(x):
    return (x & 0x7fffffff) - (x & 0x80000000)


def i_type(op, rd, f3, rs1, imm):
    return ((imm & 4095) << 20) | rs1 << 15 | f3 << 12 | rd << 7 | op


def r_type(rd, f3, a, b, f7=0):
    return f7 << 25 | b << 20 | a << 15 | f3 << 12 | rd << 7 | 0x33


def store(f3, base, src, imm):
    imm &= 4095
    return (imm >> 5) << 25 | src << 20 | base << 15 | f3 << 12 | (imm & 31) << 7 | 0x23


def branch(f3, a, b, off):
    return ((off >> 12) & 1) << 31 | ((off >> 5) & 63) << 25 | b << 20 | a << 15 | f3 << 12 | ((off >> 1) & 15) << 8 | ((off >> 11) & 1) << 7 | 0x63


def jal(rd, off):
    return ((off >> 20) & 1) << 31 | ((off >> 1) & 1023) << 21 | ((off >> 11) & 1) << 20 | ((off >> 12) & 255) << 12 | rd << 7 | 0x6f


class Program:
    def __init__(self):
        self.words = []
        self.fail_jumps = []
        self.checks = 0

    def emit(self, word):
        self.words.append(word & MASK)

    def li(self, rd, value):
        value &= MASK
        self.emit(((value + 0x800) & 0xfffff000) | rd << 7 | 0x37)
        self.emit(i_type(0x13, rd, 0, rd, value))

    def check(self, rd, expected):
        self.checks += 1
        self.li(10, self.checks)  # Failure output identifies the first failing check.
        self.li(29, expected)
        self.emit(branch(0, rd, 29, 8))
        self.fail_jumps.append(len(self.words))
        self.emit(0)

    def finish(self):
        self.li(10, 0)
        failure = len(self.words) * 4
        self.li(31, 0x80000000)
        self.emit(store(2, 31, 10, 0))
        self.emit(jal(0, 0))
        for index in self.fail_jumps:
            self.words[index] = jal(0, failure - index * 4)
        return '@00000000\n' + '\n'.join(
            ' '.join(f'{b:02x}' for b in word.to_bytes(4, 'little')) for word in self.words) + '\n'


def make_program():
    p = Program()
    rng = random.Random(2026)
    edges = [0, 1, 2, MASK, 0x80000000, 0x7fffffff, 0x80000001, 31, 32]
    pairs = [(a, b) for a in edges for b in edges]
    pairs += [(rng.getrandbits(32), rng.getrandbits(32)) for _ in range(60)]
    for a, b in pairs:
        sa, sb = signed(a), signed(b)
        quotient = 0 if b == 0 else abs(sa) // abs(sb) * (-1 if (sa < 0) != (sb < 0) else 1)
        m_results = [a*b, (sa*sb) >> 32, (sa*b) >> 32, (a*b) >> 32,
                     MASK if b == 0 else quotient, MASK if b == 0 else a//b,
                     a if b == 0 else sa-quotient*sb, a if b == 0 else a % b]
        for f3, expected in enumerate(m_results):
            p.li(1, a); p.li(2, b)
            p.emit(r_type(3, f3, 1, 2, 1))
            p.check(3, expected)
        base_results = [a+b, a << (b & 31), int(sa < sb), int(a < b), a ^ b,
                        a >> (b & 31), a | b, a & b]
        for f3, expected in enumerate(base_results):
            p.li(1, a); p.li(2, b)
            p.emit(r_type(3, f3, 1, 2))
            p.check(3, expected)
        p.emit(r_type(3, 0, 1, 2, 0x20)); p.check(3, a-b)
        p.emit(r_type(3, 5, 1, 2, 0x20)); p.check(3, sa >> (b & 31))
    for imm in [-2048, -1, 0, 1, 2047]:
        for a in edges:
            p.li(1, a)
            results = {0: a+imm, 2: int(signed(a) < imm), 3: int(a < (imm & MASK)),
                       4: a ^ (imm & MASK), 6: a | (imm & MASK), 7: a & (imm & MASK)}
            for f3, expected in results.items():
                p.emit(i_type(0x13, 3, f3, 1, imm)); p.check(3, expected)
            for amount in [0, 1, 31]:
                for f3, shift_imm, expected in [(1, amount, a << amount),
                        (5, amount, a >> amount), (5, 0x400+amount, signed(a) >> amount)]:
                    p.emit(i_type(0x13, 3, f3, 1, shift_imm)); p.check(3, expected)
    p.li(20, 0x100000)  # Data well beyond generated instructions.
    p.li(1, 0x80ff7f01)
    p.emit(store(2, 20, 1, 0))
    for f3, off, expected in [(0, 0, 1), (0, 1, 127), (0, 2, MASK), (0, 3, 0xffffff80),
            (4, 2, 255), (4, 3, 128), (1, 0, 0x7f01), (1, 2, 0xffff80ff),
            (5, 2, 0x80ff), (2, 0, 0x80ff7f01)]:
        p.emit(i_type(0x03, 3, f3, 20, off)); p.check(3, expected)
    p.li(1, 0x123456aa)
    p.emit(store(0, 20, 1, 1))
    p.emit(i_type(3, 3, 2, 20, 0)); p.check(3, 0x80ffaa01)
    p.emit(store(1, 20, 1, 2))
    p.emit(i_type(3, 3, 2, 20, 0)); p.check(3, 0x56aaaa01)
    # Cold miss, zero-filled memory, and direct-mapped data-cache collision.
    p.emit(i_type(3, 3, 2, 20, 1024)); p.check(3, 0)
    p.emit(i_type(3, 3, 2, 20, 0)); p.check(3, 0x56aaaa01)
    # All six branch conditions, taken and not taken; wrong-path stores must vanish.
    for f3 in [0, 1, 4, 5, 6, 7]:
        for a, b in [(0, 0), (MASK, 1), (1, MASK)]:
            p.li(1, a); p.li(2, b)
            taken = {0: a == b, 1: a != b, 4: signed(a) < signed(b),
                     5: signed(a) >= signed(b), 6: a < b, 7: a >= b}[f3]
            p.li(3, 0)
            p.emit(branch(f3, 1, 2, 12))
            p.emit(i_type(0x13, 3, 0, 0, 1))
            p.emit(store(2, 20, 3, 4))
            p.check(3, 0 if taken else 1)
    # Explicit wrong-path MMIO exit (initial prediction is not taken).
    p.li(31, 0x80000000)
    p.emit(branch(0, 0, 0, 12))
    p.emit(store(2, 31, 1, 0))
    p.emit(i_type(0x13, 3, 0, 0, 99))
    # JAL and JALR link and target (including clearing bit zero).
    link_pc = len(p.words)*4 + 4
    p.emit(jal(3, 8)); p.emit(store(2, 31, 1, 0)); p.check(3, link_pc)
    target = len(p.words)*4 + 16
    p.li(1, target | 1)
    link_pc = len(p.words)*4 + 4
    p.emit(i_type(0x67, 3, 0, 1, 0)); p.emit(store(2, 31, 1, 0)); p.check(3, link_pc)
    here = len(p.words)*4
    p.emit(4 << 12 | 3 << 7 | 0x17); p.check(3, here + 0x4000)
    # x0 remains zero even when used as an instruction destination.
    p.emit(i_type(0x13, 0, 0, 0, -1)); p.check(0, 0)
    return p, p.finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', type=Path, default=Path('build/sim'))
    args = parser.parse_args()
    p, image = make_program()
    with tempfile.TemporaryDirectory(prefix='rv32im-') as directory:
        path = Path(directory) / 'regression.data'
        path.write_text(image)
        for latency in [1, 10, 37]:
            run = subprocess.run([str(args.sim.resolve()), str(path), '0', '5000000', str(latency)],
                                 text=True, capture_output=True)
            print(f'latency={latency}, checks={p.checks}: {run.stdout.strip()}', flush=True)
            if run.returncode:
                raise SystemExit(run.stderr or run.stdout)


if __name__ == '__main__':
    main()
