
INDENT = "  "
REGMAP_VAR = "regmap"
ENTRY_TYPE = "regmap_t"
EXCLUDE_REGS = ["REG_COUNT"]

with open("regmap.csv", 'r') as fregmap, open("init_regmap.c", 'w') as fout:
    linestr = "void init_regmap(void) {\n"
    print(linestr, end='')
    fout.write(linestr)
    for num, line in enumerate(fregmap):
        if num > 0:
            parts = line.strip().split(',')
            enumval = int(parts[0])
            regname = parts[1]
            if regname in EXCLUDE_REGS:
                continue
            comment = parts[2]
            varname = parts[3]
            linestr = f'{INDENT}{REGMAP_VAR}[{regname}] = ({ENTRY_TYPE})(&{varname});\n'
            print(linestr, end='')
            fout.write(linestr)

    linestr = "}\n"
    fout.write(linestr)
    print(linestr, end='')