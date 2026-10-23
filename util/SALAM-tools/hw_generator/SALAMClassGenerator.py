# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


def simobject_classname(alias):
    return "".join(word.capitalize() for word in alias.split("_"))


def generate_functional_units_declaration(units, cxx_header):
    parts = []
    parts.append(
        "# AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md"
        " for details)\n\n"
    )
    parts.append("from m5.params import *\n")
    parts.append("from m5.proxy import *\n")
    parts.append("from m5.SimObject import SimObject\n\n")
    parts.append("class FunctionalUnits(SimObject):\n")
    parts.append("\t# SimObject type\n")
    parts.append("\ttype = 'FunctionalUnits'\n")
    parts.append("\t# gem5-SALAM attached header\n")
    parts.append('\tcxx_header = "' + cxx_header + '"\n\n')
    for unit in units:
        parts.append("\t" + unit + " = Param." + simobject_classname(unit))
        parts.append(
            '(Parent.any, "' + unit + ' functional unit SimObject.")\n'
        )
    parts.append(
        "\n# AUTO-GENERATED CLASSES (See"
        " util/SALAM-docs/README_SALAM.md for details)\n"
    )
    return "".join(parts)


def generate_functional_unit_declaration(
    alias,
    classname,
    header_py_path,
    *,
    stages,
    cycles,
    enum_value,
    int_size,
    int_sign,
    int_apmode,
    fp_size,
    fp_sign,
    fp_apmode,
    ptr_size,
    ptr_sign,
    ptr_apmode,
    limit,
    power_units,
    energy_units,
    time_units,
    area_units,
    fu_latency,
    internal_power,
    switch_power,
    dynamic_power,
    dynamic_energy,
    leakage_power,
    area,
    path_delay,
):
    parts = []
    parts.append("class " + classname + "(SimObject):\n")
    parts.append("\t# SimObject type\n")
    parts.append("\ttype = '" + classname + "'\n")
    parts.append("\t# gem5-SALAM attached header\n")
    parts.append('\tcxx_header = "' + header_py_path + '"\n')
    parts.append("\t#HW Params\n")
    parts.append(
        '\talias = Param.String("'
        + alias
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tstages = Param.UInt32("
        + str(stages)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tcycles = Param.UInt32("
        + str(cycles)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tenum_value = Param.UInt32("
        + str(enum_value)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tint_size = Param.String("'
        + str(int_size)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tint_sign = Param.String("'
        + str(int_sign)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tint_apmode = Param.Bool("
        + str(int_apmode)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tfp_size = Param.String("'
        + str(fp_size)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tfp_sign = Param.String("'
        + str(fp_sign)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tfp_apmode = Param.Bool("
        + str(fp_apmode)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tptr_size = Param.String("'
        + str(ptr_size)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tptr_sign = Param.String("'
        + str(ptr_sign)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tptr_apmode = Param.Bool("
        + str(ptr_apmode)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tlimit = Param.UInt32("
        + str(limit)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append("\t#Power Params\n")
    parts.append(
        '\tpower_units = Param.String("'
        + str(power_units)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tenergy_units = Param.String("'
        + str(energy_units)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\ttime_units = Param.String("'
        + str(time_units)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        '\tarea_units = Param.String("'
        + str(area_units)
        + '", "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tfu_latency = Param.UInt32("
        + str(fu_latency)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tinternal_power = Param.Float("
        + str(internal_power)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tswitch_power = Param.Float("
        + str(switch_power)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tdynamic_power = Param.Float("
        + str(dynamic_power)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tdynamic_energy = Param.Float("
        + str(dynamic_energy)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tleakage_power = Param.Float("
        + str(leakage_power)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tarea = Param.Float("
        + str(area)
        + ', "Default values set from '
        + alias
        + '.yml")\n'
    )
    parts.append(
        "\tpath_delay = Param.Float("
        + str(path_delay)
        + ', "Default values set from '
        + alias
        + '.yml")\n\n'
    )
    return "".join(parts)


def generate_fu_header(alias, classname):
    guard = alias.upper()
    return (
        "#ifndef __HWMODEL_" + guard + "_HH__\n"
        "#define __HWMODEL_" + guard + "_HH__\n\n"
        "// GENERATED FILE - DO NOT MODIFY\n\n"
        '#include "params/' + classname + '.hh"\n'
        '#include "sim/sim_object.hh"\n'
        '#include "base.hh"\n\n'
        "using namespace gem5;\n\n"
        "class "
        + classname
        + ": public SimObject, public FunctionalUnitBase\n"
        "{\n"
        "\tprivate:\n"
        "\tprotected:\n"
        "\tpublic:\n"
        "\t\t" + classname + "();\n"
        "\t\t" + classname + "(const " + classname + "Params &params);\n"
        "};\n"
        "#endif // __HWMODEL_" + guard + "_HH__"
    )


def generate_fu_source(alias, classname):
    return (
        '#include "' + alias + '.hh"\n\n'
        "// AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md"
        " for details)\n\n"
        + classname
        + "::"
        + classname
        + "(const "
        + classname
        + "Params &params) :\n"
        "\tSimObject(params),\n"
        "\tFunctionalUnitBase( params.alias,\n"
        "\t\t\tparams.stages,\n"
        "\t\t\tparams.cycles,\n"
        "\t\t\tparams.enum_value,\n"
        "\t\t\tparams.int_size,\n"
        "\t\t\tparams.int_sign,\n"
        "\t\t\tparams.int_apmode,\n"
        "\t\t\tparams.fp_size,\n"
        "\t\t\tparams.fp_sign,\n"
        "\t\t\tparams.fp_apmode,\n"
        "\t\t\tparams.ptr_size,\n"
        "\t\t\tparams.ptr_sign,\n"
        "\t\t\tparams.ptr_apmode,\n"
        "\t\t\tparams.limit,\n"
        "\t\t\tparams.power_units,\n"
        "\t\t\tparams.energy_units,\n"
        "\t\t\tparams.time_units,\n"
        "\t\t\tparams.area_units,\n"
        "\t\t\tparams.fu_latency,\n"
        "\t\t\tparams.internal_power,\n"
        "\t\t\tparams.switch_power,\n"
        "\t\t\tparams.dynamic_power,\n"
        "\t\t\tparams.dynamic_energy,\n"
        "\t\t\tparams.leakage_power,\n"
        "\t\t\tparams.area,\n"
        "\t\t\tparams.path_delay) { }\n"
    )


def generate_fu_list_header(units):
    lines = [
        "#ifndef __HWMODEL_FUNCTIONAL_UNITS_HH__\n",
        "#define __HWMODEL_FUNCTIONAL_UNITS_HH__\n\n",
        '#include "params/FunctionalUnits.hh"\n',
        '#include "sim/sim_object.hh"\n',
        "// GENERATED HEADERS - DO NOT MODIFY\n",
        '#include "functional_units/base.hh"\n',
    ]
    for unit in units:
        lines.append('#include "functional_units/' + unit + '.hh"\n')
    lines.extend(
        [
            "#include <iostream>\n",
            "#include <cstdlib>\n",
            "#include <vector>\n\n",
            "using namespace gem5;\n\n",
            "class FunctionalUnitBase;\n\n",
            "class FunctionalUnits : public SimObject\n",
            "{\n",
            "\tprivate:\n",
            "\tprotected:\n\n",
            "\tpublic:\n",
            "\t\t// GENERATED CLASS MEMBERS - DO NOT MODIFY\n",
        ]
    )
    for unit in units:
        classname = simobject_classname(unit)
        lines.append("\t\t" + classname + "* _" + unit + ";\n")
    lines.extend(
        [
            "\t\tFunctionalUnits();\n",
            "\t\t// DEFAULT CONSTRUCTOR - DO NOT MODIFY\n",
            "\t\tFunctionalUnits(const FunctionalUnitsParams &params);\n",
            "\t\t// END DEFAULT CONSTRUCTOR\n",
            "\t\tstd::vector<FunctionalUnitBase*> functional_unit_list;\n",
            "};\n",
            "#endif //__HWMODEL_FUNCTIONAL_UNITS_HH__\n",
        ]
    )
    return "".join(lines)


def generate_fu_list_source(units):
    lines = [
        '#include "functional_units.hh"\n\n',
        "// GENERATED CONSTRUCTOR - DO NOT MODIFY\n",
        "FunctionalUnits::FunctionalUnits("
        "const FunctionalUnitsParams &params) :\n",
        "    SimObject(params),\n",
    ]
    for idx, alias in enumerate(units):
        if idx < len(units) - 1:
            lines.append("\t_" + alias + "(params." + alias + "),\n")
        else:
            lines.append("\t_" + alias + "(params." + alias + ") {\n")
    for alias in units:
        lines.append("\t\tfunctional_unit_list.push_back(_" + alias + ");\n")
    lines.extend(["}\n", "// END OF GENERATED CONSTRUCTOR\n", "\n"])
    return "".join(lines)


def generate_fu_base_header():
    parts = []
    parts.append("#ifndef __HWMODEL_FUNCTIONAL_UNIT_BASE_HH__\n")
    parts.append("#define __HWMODEL_FUNCTIONAL_UNIT_BASE_HH__\n\n")
    parts.append('#include "salam/HWModeling/salam_power_model.hh"\n\n')
    parts.append("#include <cstdint>\n")
    parts.append("#include <map>\n")
    parts.append("#include <iostream>\n")
    parts.append("#include <cstdlib>\n")
    parts.append("#include <vector>\n\n")
    parts.append("class FunctionalUnitBase\n")
    parts.append("{\n")
    parts.append("\tprivate:\n")
    parts.append("\tprotected:\n")
    parts.append("\t\tstd::string _alias;\n")
    parts.append("\t\tuint32_t _stages;\n")
    parts.append("\t\tuint32_t _cycles;\n")
    parts.append("\t\tuint32_t _enum_value;\n")
    parts.append("\t\tstd::string _int_size;\n")
    parts.append("\t\tstd::string _int_sign;\n")
    parts.append("\t\tbool _int_apmode;\n")
    parts.append("\t\tstd::string _fp_size;\n")
    parts.append("\t\tstd::string _fp_sign;\n")
    parts.append("\t\tbool _fp_apmode;\n")
    parts.append("\t\tstd::string _ptr_size;\n")
    parts.append("\t\tstd::string _ptr_sign;\n")
    parts.append("\t\tbool _ptr_apmode;\n")
    parts.append("\t\tuint32_t _limit;\n")
    parts.append("\t\tstd::string _power_units;\n")
    parts.append("\t\tstd::string _energy_units;\n")
    parts.append("\t\tstd::string _time_units;\n")
    parts.append("\t\tstd::string _area_units;\n")
    parts.append("\t\tuint32_t _fu_latency;\n")
    parts.append("\t\tdouble _internal_power;\n")
    parts.append("\t\tdouble _switch_power;\n")
    parts.append("\t\tdouble _dynamic_power;\n")
    parts.append("\t\tdouble _dynamic_energy;\n")
    parts.append("\t\tdouble _leakage_power;\n")
    parts.append("\t\tdouble _area;\n")
    parts.append("\t\tdouble _path_delay;\n\n")
    parts.append("\t\tuint64_t _available = 0;\n\n")
    parts.append("\t\tuint64_t _in_use = 0;\n\n")
    parts.append("\tpublic:\n")
    parts.append("\t\tFunctionalUnitBase();\n")
    parts.append("\t\tFunctionalUnitBase( std::string alias,\n")
    parts.append("\t\t\tuint32_t stages,\n")
    parts.append("\t\t\tuint32_t cycles,\n")
    parts.append("\t\t\tuint32_t enum_value,\n")
    parts.append("\t\t\tstd::string int_size,\n")
    parts.append("\t\t\tstd::string int_sign,\n")
    parts.append("\t\t\tbool int_apmode,\n")
    parts.append("\t\t\tstd::string fp_size,\n")
    parts.append("\t\t\tstd::string fp_sign,\n")
    parts.append("\t\t\tbool fp_apmode,\n")
    parts.append("\t\t\tstd::string ptr_size,\n")
    parts.append("\t\t\tstd::string ptr_sign,\n")
    parts.append("\t\t\tbool ptr_apmode,\n")
    parts.append("\t\t\tuint32_t limit,\n")
    parts.append("\t\t\tstd::string power_units,\n")
    parts.append("\t\t\tstd::string energy_units,\n")
    parts.append("\t\t\tstd::string time_units,\n")
    parts.append("\t\t\tstd::string area_units,\n")
    parts.append("\t\t\tuint32_t fu_latency,\n")
    parts.append("\t\t\tdouble internal_power,\n")
    parts.append("\t\t\tdouble switch_power,\n")
    parts.append("\t\t\tdouble dynamic_power,\n")
    parts.append("\t\t\tdouble dynamic_energy,\n")
    parts.append("\t\t\tdouble leakage_power,\n")
    parts.append("\t\t\tdouble area,\n")
    parts.append("\t\t\tdouble path_delay) :\n")
    parts.append("\t\t\t_alias(alias),\n")
    parts.append("\t\t\t_stages(stages),\n")
    parts.append("\t\t\t_cycles(cycles),\n")
    parts.append("\t\t\t_enum_value(enum_value),\n")
    parts.append("\t\t\t_int_size(int_size),\n")
    parts.append("\t\t\t_int_sign(int_sign),\n")
    parts.append("\t\t\t_int_apmode(int_apmode),\n")
    parts.append("\t\t\t_fp_size(fp_size),\n")
    parts.append("\t\t\t_fp_sign(fp_sign),\n")
    parts.append("\t\t\t_fp_apmode(fp_apmode),\n")
    parts.append("\t\t\t_ptr_size(ptr_size),\n")
    parts.append("\t\t\t_ptr_sign(ptr_sign),\n")
    parts.append("\t\t\t_ptr_apmode(ptr_apmode),\n")
    parts.append("\t\t\t_limit(limit),\n")
    parts.append("\t\t\t_power_units(power_units),\n")
    parts.append("\t\t\t_energy_units(energy_units),\n")
    parts.append("\t\t\t_time_units(time_units),\n")
    parts.append("\t\t\t_area_units(area_units),\n")
    parts.append("\t\t\t_fu_latency(fu_latency),\n")
    parts.append("\t\t\t_internal_power(internal_power),\n")
    parts.append("\t\t\t_switch_power(switch_power),\n")
    parts.append("\t\t\t_dynamic_power(dynamic_power),\n")
    parts.append("\t\t\t_dynamic_energy(dynamic_energy),\n")
    parts.append("\t\t\t_leakage_power(leakage_power),\n")
    parts.append("\t\t\t_area(area),\n")
    parts.append("\t\t\t_path_delay(path_delay) { }\n")
    parts.append("\t\tstd::string get_alias() { return _alias; }\n")
    parts.append("\t\tuint32_t get_stages() { return _stages; }\n")
    parts.append("\t\tuint32_t get_cycles() { return _cycles; }\n")
    parts.append("\t\tuint32_t get_enum_value() { return _enum_value; }\n")
    parts.append("\t\tstd::string get_int_size() { return _int_size; }\n")
    parts.append("\t\tstd::string get_int_sign() { return _int_sign; }\n")
    parts.append("\t\tbool get_int_apmode() { return _int_apmode; }\n")
    parts.append("\t\tstd::string get_fp_size() { return _fp_size; }\n")
    parts.append("\t\tstd::string get_fp_sign() { return _fp_sign; }\n")
    parts.append("\t\tbool get_fp_apmode() { return _fp_apmode; }\n")
    parts.append("\t\tstd::string get_ptr_size() { return _ptr_size; }\n")
    parts.append("\t\tstd::string get_ptr_sign() { return _ptr_sign; }\n")
    parts.append("\t\tbool get_ptr_apmode() { return _ptr_apmode; }\n")
    parts.append("\t\tuint32_t get_limit() { return _limit; }\n")
    parts.append(
        "\t\tstd::string get_power_units() { return _power_units; }\n"
    )
    parts.append(
        "\t\tstd::string get_energy_units()" "{ return _energy_units; }\n"
    )
    parts.append("\t\tstd::string get_time_units() { return _time_units; }\n")
    parts.append("\t\tstd::string get_area_units() { return _area_units; }\n")
    parts.append("\t\tuint32_t get_fu_latency() { return _fu_latency; }\n")
    parts.append(
        "\t\tdouble get_internal_power() { return _internal_power; }\n"
    )
    parts.append("\t\tdouble get_switch_power() { return _switch_power; }\n")
    parts.append("\t\tdouble get_dynamic_power() { return _dynamic_power; }\n")
    parts.append(
        "\t\tdouble get_dynamic_energy() { return _dynamic_energy; }\n"
    )
    parts.append("\t\tdouble get_leakage_power() { return _leakage_power; }\n")
    parts.append("\t\tdouble get_area() { return _area; }\n")
    parts.append("\t\tdouble get_path_delay() { return _path_delay; }\n")
    parts.append("\t\tbool is_available()\n")
    parts.append("\t\t{\n")
    parts.append("\t\t\treturn (_available == 0 || _in_use < _available);\n")
    parts.append("\t\t}\n")
    parts.append("\t\tvoid use_functional_unit() { _in_use++; }\n")
    parts.append("\t\tvoid clear_functional_unit() { _in_use--; }\n")
    parts.append(
        "\t\tvoid set_functional_unit_limit(uint64_t available)"
        "{ _available = available; }\n"
    )
    parts.append("\t\tvoid inc_functional_unit_limit() { _available++; }\n")
    parts.append(
        "\t\tuint64_t get_functional_unit_limit()" "{ return _available; }\n\n"
    )
    parts.append("\t\tuint64_t get_in_use() { return _in_use; }\n")
    parts.append("};\n")
    parts.append("#endif // __HWMODEL_FUNCTIONAL_UNIT_BASE_HH__")
    return "".join(parts)


def generate_instruction_declaration(
    inst_name,
    *,
    functional_unit,
    functional_unit_limit,
    opcode_num,
    runtime_cycles,
):
    classname = simobject_classname(inst_name)
    return (
        "class "
        + classname
        + "(SimObject):\n"
        + "\t# SimObject type\n"
        + "\ttype = '"
        + classname
        + "'"
        + "\t# gem5-SALAM attached header\n"
        + '\tcxx_header = "salam/HWModeling/instructions/'
        + str(inst_name)
        + '.hh"\n'
        + "\t# Instruction params\n"
        + "\tfunctional_unit = Param.UInt32("
        + str(functional_unit)
        + ', "Default functional unit assignment.")\n'
        + "\tfunctional_unit_limit = Param.UInt32("
        + str(functional_unit_limit)
        + ', "Default functional unit limit.")\n'
        + "\topcode_num = Param.UInt32("
        + str(opcode_num)
        + ', "Default instruction llvm enum opcode value.")\n'
        + "\truntime_cycles = Param.UInt32("
        + str(runtime_cycles)
        + ', "Default instruction runtime cycles.")\n\n'
    )


def generate_instconfig_declaration(inst_names):
    parts = []
    parts.append("class InstConfig(SimObject):\n")
    parts.append("\t# SimObject type\n")
    parts.append("\ttype = 'InstConfig'\n")
    parts.append("\t# gem5-SALAM attached header\n")
    parts.append(
        "\tcxx_header =" '"salam/HWModeling/instruction_config.hh"\n\n'
    )
    for inst_name in inst_names:
        parts.append(
            "\t"
            + str(inst_name)
            + " = Param."
            + simobject_classname(inst_name)
            + '(Parent.any, "'
            + str(inst_name)
            + ' instruction SimObject")\n'
        )
    return "".join(parts)


def generate_instruction_config_preamble():
    return (
        "# AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md"
        " for details)\n\n"
        "from m5.params import *\n"
        "from m5.proxy import *\n"
        "from m5.SimObject import SimObject\n\n"
        "\n# AUTO-GENERATED CLASSES (See"
        " util/SALAM-docs/README_SALAM.md for details)\n"
    )


def generate_instruction_config_header(inst_names):
    lines = [
        "#ifndef __HWMODEL_INSTRUCTION_CONFIG_HH__\n",
        "#define __HWMODEL_INSTRUCTION_CONFIG_HH__\n\n",
        '#include "params/InstConfig.hh"\n',
        '#include "sim/sim_object.hh"\n',
        "// GENERATED HEADERS - DO NOT MODIFY\n",
        '#include "instructions/base.hh"\n',
    ]
    for inst in inst_names:
        lines.append('#include "instructions/' + inst + '.hh"\n')
    lines.extend(
        [
            "#include <iostream>\n",
            "#include <cstdlib>\n",
            "#include <vector>\n\n",
            "using namespace gem5;\n\n",
            "class InstConfigBase;\n\n",
            "class InstConfig : public SimObject\n",
            "{\n",
            "\tprivate:\n",
            "\tprotected:\n\n",
            "\tpublic:\n",
            "\t\t// GENERATED CLASS MEMBERS - DO NOT MODIFY\n",
        ]
    )
    for inst in inst_names:
        lines.append("\t\t" + simobject_classname(inst) + "* _" + inst + ";\n")
    lines.extend(
        [
            "\t\tInstConfig();\n",
            "\t\t// DEFAULT CONSTRUCTOR - DO NOT MODIFY\n",
            "\t\tInstConfig(const InstConfigParams &params);\n",
            "\t\t// END DEFAULT CONSTRUCTOR\n",
            "\t\tstd::vector<InstConfigBase*> inst_list;",
            "};\n",
            "#endif //__INSTRUCTION_CONFIG_HH__\n",
        ]
    )
    return "".join(lines)


def generate_instruction_config_source(inst_names):
    names = list(inst_names)
    lines = [
        '#include "instruction_config.hh"\n\n',
        "// GENERATED CONSTRUCTOR - DO NOT MODIFY\n",
        "InstConfig::InstConfig(const InstConfigParams &params) :\n",
        "\tSimObject(params),\n",
    ]
    for idx, inst in enumerate(names):
        if idx < len(names) - 1:
            lines.append("\t_" + inst + "(params." + inst + "),\n")
        else:
            lines.append("\t_" + inst + "(params." + inst + ") {\n")
    for inst in names:
        lines.append("\tinst_list.push_back(_" + inst + ");\n")
    lines.append("}\n")
    lines.append("// END OF GENERATED CONSTRUCTOR\n")
    lines.append("\n")
    return "".join(lines)


def generate_instruction_unit_header(inst_name, classname):
    guard = str(inst_name).upper()
    return (
        "#ifndef __HWMODEL_"
        + guard
        + "_HH__\n"
        + "#define __HWMODEL_"
        + guard
        + "_HH__\n\n"
        + "// GENERATED FILE - DO NOT MODIFY\n\n"
        + '#include "params/'
        + classname
        + '.hh"\n'
        + '#include "sim/sim_object.hh"\n'
        + '#include "base.hh"\n\n'
        + "using namespace gem5;\n\n"
        + "class "
        + classname
        + ": public SimObject, public InstConfigBase\n"
        + "{\n"
        + "\tprivate:\n"
        + "\tprotected:\n"
        + "\tpublic:\n"
        + "\t\t"
        + classname
        + "();\n"
        + "\t\t"
        + classname
        + "(const "
        + classname
        + "Params &params);\n"
        + "};\n"
        + "#endif // __HWMODEL_"
        + guard
        + "_HH__"
    )


def generate_instruction_unit_source(inst_name, classname):
    return (
        '#include "'
        + str(inst_name)
        + '.hh"\n\n'
        + "// AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md"
        + " for details)\n\n"
        + classname
        + "::"
        + classname
        + "(const "
        + classname
        + "Params &params) :\n"
        + "\tSimObject(params),\n"
        + "\tInstConfigBase( params.functional_unit,\n"
        + "\t\t\tparams.functional_unit_limit,\n"
        + "\t\t\tparams.opcode_num,\n"
        + "\t\t\tparams.runtime_cycles) { }\n"
    )


def generate_instruction_base_header():
    return (
        "#ifndef __HWMODEL_INST_CONFIG_BASE_HH__\n"
        "#define __HWMODEL_INST_CONFIG_BASE_HH__\n\n"
        "#include <cstdint>\n"
        "#include <map>\n"
        "#include <iostream>\n"
        "#include <cstdlib>\n"
        "#include <vector>\n\n"
        "class InstConfigBase\n"
        "{\n"
        "\tprivate:\n"
        "\tprotected:\n"
        "\t\tuint32_t _functional_unit;\n"
        "\t\tuint32_t _functional_unit_limit;\n"
        "\t\tuint32_t _opcode_num;\n"
        "\t\tuint32_t _runtime_cycles;\n"
        "\tpublic:\n"
        "\t\tInstConfigBase();\n"
        "\t\tInstConfigBase( uint32_t functional_unit,\n"
        "\t\t\tuint32_t functional_unit_limit,\n"
        "\t\t\tuint32_t opcode_num,\n"
        "\t\t\tuint32_t runtime_cycles) :\n"
        "\t\t\t_functional_unit(functional_unit),\n"
        "\t\t\t_functional_unit_limit(functional_unit_limit),\n"
        "\t\t\t_opcode_num(opcode_num),\n"
        "\t\t\t_runtime_cycles(runtime_cycles) { }\n"
        "\t\tuint32_t get_functional_unit()"
        "{ return _functional_unit; }\n"
        "\t\tuint32_t get_functional_unit_limit()"
        "{ return _functional_unit_limit; }\n"
        "\t\tuint32_t get_opcode_num() { return _opcode_num; }\n"
        "\t\tuint32_t get_runtime_cycles()"
        "{ return _runtime_cycles; }\n"
        "};\n"
        "#endif // __HWMODEL_INST_CONFIG_BASE_HH__"
    )
