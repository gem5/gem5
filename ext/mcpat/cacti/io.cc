/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/



#include <fstream>
#include <iostream>
#include <sstream>

#include "Ucache.h"
#include "arbiter.h"
#include "area.h"
#include "basic_circuit.h"
#include "crossbar.h"
#include "io.h"
#include "nuca.h"
#include "parameter.h"
//#include "highradix.h"

using namespace std;


/* Parses "cache.cfg" file */
void
InputParameter::parse_cfg(const string & in_file) {
    FILE *fp = fopen(in_file.c_str(), "r");
    char line[5000];
    char jk[5000];
    char temp_var[5000];

    if (!fp) {
        cout << in_file << " is missing!\n";
        exit(-1);
    }

    while (fscanf(fp, "%[^\n]\n", line) != EOF) {

        if (!strncmp("-size", line, strlen("-size"))) {
            sscanf(line, "-size %[(:-~)*]%u", jk, &(cache_sz));
            continue;
        }

        if (!strncmp("-page size", line, strlen("-page size"))) {
            sscanf(line, "-page size %[(:-~)*]%u", jk, &(page_sz_bits));
            continue;
        }

        if (!strncmp("-burst length", line, strlen("-burst length"))) {
            sscanf(line, "-burst %[(:-~)*]%u", jk, &(burst_len));
            continue;
        }

        if (!strncmp("-internal prefetch width", line, strlen("-internal prefetch width"))) {
            sscanf(line, "-internal prefetch %[(:-~)*]%u", jk, &(int_prefetch_w));
            continue;
        }

        if (!strncmp("-block", line, strlen("-block"))) {
            sscanf(line, "-block size (bytes) %d", &(line_sz));
            continue;
        }

        if (!strncmp("-associativity", line, strlen("-associativity"))) {
            sscanf(line, "-associativity %d", &(assoc));
            continue;
        }

        if (!strncmp("-read-write", line, strlen("-read-write"))) {
            sscanf(line, "-read-write port %d", &(num_rw_ports));
            continue;
        }

        if (!strncmp("-exclusive read", line, strlen("exclusive read"))) {
            sscanf(line, "-exclusive read port %d", &(num_rd_ports));
            continue;
        }

        if (!strncmp("-exclusive write", line, strlen("-exclusive write"))) {
            sscanf(line, "-exclusive write port %d", &(num_wr_ports));
            continue;
        }

        if (!strncmp("-single ended", line, strlen("-single ended"))) {
            sscanf(line, "-single %[(:-~)*]%d", jk,
                   &(num_se_rd_ports));
            continue;
        }

        if (!strncmp("-search", line, strlen("-search"))) {
            sscanf(line, "-search port %d", &(num_search_ports));
            continue;
        }

        if (!strncmp("-UCA bank", line, strlen("-UCA bank"))) {
            sscanf(line, "-UCA bank%[((:-~)| )*]%d", jk, &(nbanks));
            continue;
        }

        if (!strncmp("-technology", line, strlen("-technology"))) {
            sscanf(line, "-technology (u) %lf", &(F_sz_um));
            F_sz_nm = F_sz_um * 1000;
            continue;
        }

        if (!strncmp("-output/input", line, strlen("-output/input"))) {
            sscanf(line, "-output/input bus %[(:-~)*]%d", jk, &(out_w));
            continue;
        }

        if (!strncmp("-operating temperature", line, strlen("-operating temperature"))) {
            sscanf(line, "-operating temperature %[(:-~)*]%d", jk, &(temp));
            continue;
        }

        if (!strncmp("-cache type", line, strlen("-cache type"))) {
            sscanf(line, "-cache type%[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("cache", temp_var, sizeof("cache"))) {
                is_cache = true;
            } else {
                is_cache = false;
            }

            if (!strncmp("main memory", temp_var, sizeof("main memory"))) {
                is_main_mem = true;
            } else {
                is_main_mem = false;
            }

            if (!strncmp("cam", temp_var, sizeof("cam"))) {
                pure_cam = true;
            } else {
                pure_cam = false;
            }

            if (!strncmp("ram", temp_var, sizeof("ram"))) {
                pure_ram = true;
            } else {
                if (!is_main_mem)
                    pure_ram = false;
                else
                    pure_ram = true;
            }

            continue;
        }


        if (!strncmp("-tag size", line, strlen("-tag size"))) {
            sscanf(line, "-tag size%[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("default", temp_var, sizeof("default"))) {
                specific_tag = false;
                tag_w = 42; /* the acutal value is calculated
                     * later based on the cache size, bank count, and associativity
                     */
            } else {
                specific_tag = true;
                sscanf(line, "-tag size (b) %d", &(tag_w));
            }
            continue;
        }

        if (!strncmp("-access mode", line, strlen("-access mode"))) {
            sscanf(line, "-access %[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("fast", temp_var, strlen("fast"))) {
                access_mode = 2;
            } else if (!strncmp("sequential", temp_var, strlen("sequential"))) {
                access_mode = 1;
            } else if (!strncmp("normal", temp_var, strlen("normal"))) {
                access_mode = 0;
            } else {
                cout << "ERROR: Invalid access mode!\n";
                exit(0);
            }
            continue;
        }

        if (!strncmp("-Data array cell type", line,
                     strlen("-Data array cell type"))) {
            sscanf(line, "-Data array cell type %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("itrs-hp", temp_var, strlen("itrs-hp"))) {
                data_arr_ram_cell_tech_type = 0;
            } else if (!strncmp("itrs-lstp", temp_var, strlen("itrs-lstp"))) {
                data_arr_ram_cell_tech_type = 1;
            } else if (!strncmp("itrs-lop", temp_var, strlen("itrs-lop"))) {
                data_arr_ram_cell_tech_type = 2;
            } else if (!strncmp("lp-dram", temp_var, strlen("lp-dram"))) {
                data_arr_ram_cell_tech_type = 3;
            } else if (!strncmp("comm-dram", temp_var, strlen("comm-dram"))) {
                data_arr_ram_cell_tech_type = 4;
            } else {
                cout << "ERROR: Invalid type!\n";
                exit(0);
            }
            continue;
        }

        if (!strncmp("-Data array peripheral type", line, strlen("-Data array peripheral type"))) {
            sscanf(line, "-Data array peripheral type %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("itrs-hp", temp_var, strlen("itrs-hp"))) {
                data_arr_peri_global_tech_type = 0;
            } else if (!strncmp("itrs-lstp", temp_var, strlen("itrs-lstp"))) {
                data_arr_peri_global_tech_type = 1;
            } else if (!strncmp("itrs-lop", temp_var, strlen("itrs-lop"))) {
                data_arr_peri_global_tech_type = 2;
            } else {
                cout << "ERROR: Invalid type!\n";
                exit(0);
            }
            continue;
        }

        if (!strncmp("-Tag array cell type", line, strlen("-Tag array cell type"))) {
            sscanf(line, "-Tag array cell type %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("itrs-hp", temp_var, strlen("itrs-hp"))) {
                tag_arr_ram_cell_tech_type = 0;
            } else if (!strncmp("itrs-lstp", temp_var, strlen("itrs-lstp"))) {
                tag_arr_ram_cell_tech_type = 1;
            } else if (!strncmp("itrs-lop", temp_var, strlen("itrs-lop"))) {
                tag_arr_ram_cell_tech_type = 2;
            } else if (!strncmp("lp-dram", temp_var, strlen("lp-dram"))) {
                tag_arr_ram_cell_tech_type = 3;
            } else if (!strncmp("comm-dram", temp_var, strlen("comm-dram"))) {
                tag_arr_ram_cell_tech_type = 4;
            } else {
                cout << "ERROR: Invalid type!\n";
                exit(0);
            }
            continue;
        }

        if (!strncmp("-Tag array peripheral type", line, strlen("-Tag array peripheral type"))) {
            sscanf(line, "-Tag array peripheral type %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("itrs-hp", temp_var, strlen("itrs-hp"))) {
                tag_arr_peri_global_tech_type = 0;
            } else if (!strncmp("itrs-lstp", temp_var, strlen("itrs-lstp"))) {
                tag_arr_peri_global_tech_type = 1;
            } else if (!strncmp("itrs-lop", temp_var, strlen("itrs-lop"))) {
                tag_arr_peri_global_tech_type = 2;
            } else {
                cout << "ERROR: Invalid type!\n";
                exit(0);
            }
            continue;
        }
        if (!strncmp("-design", line, strlen("-design"))) {
            sscanf(line, "-%[((:-~)| |,)*]%d:%d:%d:%d:%d", jk,
                   &(delay_wt), &(dynamic_power_wt),
                   &(leakage_power_wt),
                   &(cycle_time_wt), &(area_wt));
            continue;
        }

        if (!strncmp("-deviate", line, strlen("-deviate"))) {
            sscanf(line, "-%[((:-~)| |,)*]%d:%d:%d:%d:%d", jk,
                   &(delay_dev), &(dynamic_power_dev),
                   &(leakage_power_dev),
                   &(cycle_time_dev), &(area_dev));
            continue;
        }

        if (!strncmp("-Optimize", line, strlen("-Optimize"))) {
            sscanf(line, "-Optimize  %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("ED^2", temp_var, strlen("ED^2"))) {
                ed = 2;
            } else if (!strncmp("ED", temp_var, strlen("ED"))) {
                ed = 1;
            } else {
                ed = 0;
            }
        }

        if (!strncmp("-NUCAdesign", line, strlen("-NUCAdesign"))) {
            sscanf(line, "-%[((:-~)| |,)*]%d:%d:%d:%d:%d", jk,
                   &(delay_wt_nuca), &(dynamic_power_wt_nuca),
                   &(leakage_power_wt_nuca),
                   &(cycle_time_wt_nuca), &(area_wt_nuca));
            continue;
        }

        if (!strncmp("-NUCAdeviate", line, strlen("-NUCAdeviate"))) {
            sscanf(line, "-%[((:-~)| |,)*]%d:%d:%d:%d:%d", jk,
                   &(delay_dev_nuca), &(dynamic_power_dev_nuca),
                   &(leakage_power_dev_nuca),
                   &(cycle_time_dev_nuca), &(area_dev_nuca));
            continue;
        }

        if (!strncmp("-Cache model", line, strlen("-cache model"))) {
            sscanf(line, "-Cache model %[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("UCA", temp_var, strlen("UCA"))) {
                nuca = 0;
            } else {
                nuca = 1;
            }
            continue;
        }

        if (!strncmp("-NUCA bank", line, strlen("-NUCA bank"))) {
            sscanf(line, "-NUCA bank count %d", &(nuca_bank_count));

            if (nuca_bank_count != 0) {
                force_nuca_bank = 1;
            }
            continue;
        }

        if (!strncmp("-Wire inside mat", line, strlen("-Wire inside mat"))) {
            sscanf(line, "-Wire%[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("global", temp_var, strlen("global"))) {
                wire_is_mat_type = 2;
                continue;
            } else if (!strncmp("local", temp_var, strlen("local"))) {
                wire_is_mat_type = 0;
                continue;
            } else {
                wire_is_mat_type = 1;
                continue;
            }
        }

        if (!strncmp("-Wire outside mat", line, strlen("-Wire outside mat"))) {
            sscanf(line, "-Wire%[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("global", temp_var, strlen("global"))) {
                wire_os_mat_type = 2;
            } else {
                wire_os_mat_type = 1;
            }
            continue;
        }

        if (!strncmp("-Interconnect projection", line, strlen("-Interconnect projection"))) {
            sscanf(line, "-Interconnect projection%[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("aggressive", temp_var, strlen("aggressive"))) {
                ic_proj_type = 0;
            } else {
                ic_proj_type = 1;
            }
            continue;
        }

        if (!strncmp("-Wire signalling", line, strlen("-wire signalling"))) {
            sscanf(line, "-Wire%[^\"]\"%[^\"]\"", jk, temp_var);

            if (!strncmp("default", temp_var, strlen("default"))) {
                force_wiretype = 0;
                wt = Global;
            } else if (!(strncmp("Global_10", temp_var, strlen("Global_10")))) {
                force_wiretype = 1;
                wt = Global_10;
            } else if (!(strncmp("Global_20", temp_var, strlen("Global_20")))) {
                force_wiretype = 1;
                wt = Global_20;
            } else if (!(strncmp("Global_30", temp_var, strlen("Global_30")))) {
                force_wiretype = 1;
                wt = Global_30;
            } else if (!(strncmp("Global_5", temp_var, strlen("Global_5")))) {
                force_wiretype = 1;
                wt = Global_5;
            } else if (!(strncmp("Global", temp_var, strlen("Global")))) {
                force_wiretype = 1;
                wt = Global;
            } else {
                wt = Low_swing;
                force_wiretype = 1;
            }
            continue;
        }



        if (!strncmp("-Core", line, strlen("-Core"))) {
            sscanf(line, "-Core count %d\n", &(cores));
            if (cores > 16) {
                printf("No. of cores should be less than 16!\n");
            }
            continue;
        }

        if (!strncmp("-Cache level", line, strlen("-Cache level"))) {
            sscanf(line, "-Cache l%[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("L2", temp_var, strlen("L2"))) {
                cache_level = 0;
            } else {
                cache_level = 1;
            }
        }

        if (!strncmp("-Print level", line, strlen("-Print level"))) {
            sscanf(line, "-Print l%[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("DETAILED", temp_var, strlen("DETAILED"))) {
                print_detail = 1;
            } else {
                print_detail = 0;
            }

        }
        if (!strncmp("-Add ECC", line, strlen("-Add ECC"))) {
            sscanf(line, "-Add ECC %[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("true", temp_var, strlen("true"))) {
                add_ecc_b_ = true;
            } else {
                add_ecc_b_ = false;
            }
        }

        if (!strncmp("-Print input parameters", line, strlen("-Print input parameters"))) {
            sscanf(line, "-Print input %[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("true", temp_var, strlen("true"))) {
                print_input_args = true;
            } else {
                print_input_args = false;
            }
        }

        if (!strncmp("-Force cache config", line, strlen("-Force cache config"))) {
            sscanf(line, "-Force cache %[^\"]\"%[^\"]\"", jk, temp_var);
            if (!strncmp("true", temp_var, strlen("true"))) {
                force_cache_config = true;
            } else {
                force_cache_config = false;
            }
        }

        if (!strncmp("-Ndbl", line, strlen("-Ndbl"))) {
            sscanf(line, "-Ndbl %d\n", &(ndbl));
            continue;
        }
        if (!strncmp("-Ndwl", line, strlen("-Ndwl"))) {
            sscanf(line, "-Ndwl %d\n", &(ndwl));
            continue;
        }
        if (!strncmp("-Nspd", line, strlen("-Nspd"))) {
            sscanf(line, "-Nspd %d\n", &(nspd));
            continue;
        }
        if (!strncmp("-Ndsam1", line, strlen("-Ndsam1"))) {
            sscanf(line, "-Ndsam1 %d\n", &(ndsam1));
            continue;
        }
        if (!strncmp("-Ndsam2", line, strlen("-Ndsam2"))) {
            sscanf(line, "-Ndsam2 %d\n", &(ndsam2));
            continue;
        }
        if (!strncmp("-Ndcm", line, strlen("-Ndcm"))) {
            sscanf(line, "-Ndcm %d\n", &(ndcm));
            continue;
        }

    }
    rpters_in_htree = true;
    fclose(fp);
}

void
InputParameter::display_ip() {
    cout << "Cache size                    : " << cache_sz << endl;
    cout << "Block size                    : " << line_sz << endl;
    cout << "Associativity                 : " << assoc << endl;
    cout << "Read only ports               : " << num_rd_ports << endl;
    cout << "Write only ports              : " << num_wr_ports << endl;
    cout << "Read write ports              : " << num_rw_ports << endl;
    cout << "Single ended read ports       : " << num_se_rd_ports << endl;
    if (fully_assoc || pure_cam) {
        cout << "Search ports                  : " << num_search_ports << endl;
    }
    cout << "Cache banks (UCA)             : " << nbanks << endl;
    cout << "Technology                    : " << F_sz_um << endl;
    cout << "Temperature                   : " << temp << endl;
    cout << "Tag size                      : " << tag_w << endl;
    if (is_cache) {
        cout << "array type                    : " << "Cache" << endl;
    }
    if (pure_ram) {
        cout << "array type                    : " << "Scratch RAM" << endl;
    }
    if (pure_cam) {
        cout << "array type                    : " << "CAM" << endl;
    }
    cout << "Model as memory               : " << is_main_mem << endl;
    cout << "Access mode                   : " << access_mode << endl;
    cout << "Data array cell type          : " << data_arr_ram_cell_tech_type << endl;
    cout << "Data array peripheral type    : " << data_arr_peri_global_tech_type << endl;
    cout << "Tag array cell type           : " << tag_arr_ram_cell_tech_type << endl;
    cout << "Tag array peripheral type     : " << tag_arr_peri_global_tech_type << endl;
    cout << "Optimization target           : " << ed << endl;
    cout << "Design objective (UCA wt)     : " << delay_wt << " "
         << dynamic_power_wt << " " << leakage_power_wt << " " << cycle_time_wt
         << " " << area_wt << endl;
    cout << "Design objective (UCA dev)    : " << delay_dev << " "
         << dynamic_power_dev << " " << leakage_power_dev << " " << cycle_time_dev
         << " " << area_dev << endl;
    if (nuca) {
        cout << "Cores                         : " << cores << endl;


        cout << "Design objective (NUCA wt)    : " << delay_wt_nuca << " "
             << dynamic_power_wt_nuca << " " << leakage_power_wt_nuca << " " << cycle_time_wt_nuca
             << " " << area_wt_nuca << endl;
        cout << "Design objective (NUCA dev)   : " << delay_dev_nuca << " "
             << dynamic_power_dev_nuca << " " << leakage_power_dev_nuca << " " << cycle_time_dev_nuca
             << " " << area_dev_nuca << endl;
    }
    cout << "Cache model                   : " << nuca << endl;
    cout << "Nuca bank                     : " << nuca_bank_count << endl;
    cout << "Wire inside mat               : " << wire_is_mat_type << endl;
    cout << "Wire outside mat              : " << wire_os_mat_type << endl;
    cout << "Interconnect projection       : " << ic_proj_type << endl;
    cout << "Wire signalling               : " << force_wiretype << endl;
    cout << "Print level                   : " << print_detail << endl;
    cout << "ECC overhead                  : " << add_ecc_b_ << endl;
    cout << "Page size                     : " << page_sz_bits << endl;
    cout << "Burst length                  : " << burst_len << endl;
    cout << "Internal prefetch width       : " << int_prefetch_w << endl;
    cout << "Force cache config            : " << g_ip->force_cache_config << endl;
    if (g_ip->force_cache_config) {
        cout << "Ndwl                          : " << g_ip->ndwl << endl;
        cout << "Ndbl                          : " << g_ip->ndbl << endl;
        cout << "Nspd                          : " << g_ip->nspd << endl;
        cout << "Ndcm                          : " << g_ip->ndcm << endl;
        cout << "Ndsam1                        : " << g_ip->ndsam1 << endl;
        cout << "Ndsam2                        : " << g_ip->ndsam2 << endl;
    }
}



powerComponents operator+(const powerComponents & x, const powerComponents & y) {
    powerComponents z;

    z.dynamic = x.dynamic + y.dynamic;
    z.leakage = x.leakage + y.leakage;
    z.gate_leakage  = x.gate_leakage  + y.gate_leakage;
    z.short_circuit = x.short_circuit + y.short_circuit;
    z.longer_channel_leakage = x.longer_channel_leakage + y.longer_channel_leakage;

    return z;
}

powerComponents operator*(const powerComponents & x, double const * const y) {
    powerComponents z;

    z.dynamic = x.dynamic * y[0];
    z.leakage = x.leakage * y[1];
    z.gate_leakage  = x.gate_leakage * y[2];
    z.short_circuit = x.short_circuit * y[3];
    //longer channel leakage has the same behavior as normal leakage
    z.longer_channel_leakage = x.longer_channel_leakage * y[1];

    return z;
}


powerDef operator+(const powerDef & x, const powerDef & y) {
    powerDef z;

    z.readOp   = x.readOp  + y.readOp;
    z.writeOp  = x.writeOp + y.writeOp;
    z.searchOp = x.searchOp + y.searchOp;
    return z;
}

powerDef operator*(const powerDef & x, double const * const y) {
    powerDef z;

    z.readOp   = x.readOp * y;
    z.writeOp  = x.writeOp * y;
    z.searchOp = x.searchOp * y;
    return z;
}

uca_org_t cacti_interface(const string & infile_name) {

    uca_org_t fin_res;
    //uca_org_t result;
    fin_res.valid = false;

    g_ip = new InputParameter();
    g_ip->parse_cfg(infile_name);
    if (!g_ip->error_checking(infile_name))
        exit(0);
    if (g_ip->print_input_args)
        g_ip->display_ip();

    init_tech_params(g_ip->F_sz_um, false);
    Wire winit; // Do not delete this line. It initializes wires.


//  For HighRadix Only
//  ////  Wire wirea(g_ip->wt, 1000);
//  ////  wirea.print_wire();
//  ////  cout << "Wire Area " << wirea.area.get_area() << " sq. u" << endl;
//  //  winit.print_wire();
//  //
//    HighRadix *hr;
//      hr = new HighRadix();
//      hr->compute_power();
//      hr->print_router();
//    exit(0);
//
//    double sub_switch_sz = 2;
//    double rows = 32;
//    for (int i=0; i<6; i++) {
//      sub_switch_sz = pow(2, i);
//      rows = 64/sub_switch_sz;
//      hr = new HighRadix(sub_switch_sz, rows, .8/* freq */, 64, 2, 64, 0.7);
//      hr->compute_power();
//      hr->print_router();
//      delete hr;
//    }
//  //  HighRadix yarc;
//  //  yarc.compute_power();
//  //  yarc.print_router();
//    winit.print_wire();
//    exit(0);
//  For HighRadix Only End

    if (g_ip->nuca == 1) {
        Nuca n(&g_tp.peri_global);
        n.sim_nuca();
    }
    g_ip->display_ip();
    solve(&fin_res);

    output_UCA(&fin_res);
    output_data_csv(fin_res);

    delete (g_ip);
    return fin_res;
}

//cacti6.5's plain interface, please keep !!!
uca_org_t cacti_interface(
    int cache_size,
    int line_size,
    int associativity,
    int rw_ports,
    int excl_read_ports,
    int excl_write_ports,
    int single_ended_read_ports,
    int banks,
    double tech_node, // in nm
    int page_sz,
    int burst_length,
    int pre_width,
    int output_width,
    int specific_tag,
    int tag_width,
    int access_mode, //0 normal, 1 seq, 2 fast
    int cache, //scratch ram or cache
    int main_mem,
    int obj_func_delay,
    int obj_func_dynamic_power,
    int obj_func_leakage_power,
    int obj_func_area,
    int obj_func_cycle_time,
    int dev_func_delay,
    int dev_func_dynamic_power,
    int dev_func_leakage_power,
    int dev_func_area,
    int dev_func_cycle_time,
    int ed_ed2_none, // 0 - ED, 1 - ED^2, 2 - use weight and deviate
    int temp,
    int wt, //0 - default(search across everything), 1 - global, 2 - 5% delay penalty, 3 - 10%, 4 - 20 %, 5 - 30%, 6 - low-swing
    int data_arr_ram_cell_tech_flavor_in, // 0-4
    int data_arr_peri_global_tech_flavor_in,
    int tag_arr_ram_cell_tech_flavor_in,
    int tag_arr_peri_global_tech_flavor_in,
    int interconnect_projection_type_in, // 0 - aggressive, 1 - normal
    int wire_inside_mat_type_in,
    int wire_outside_mat_type_in,
    int is_nuca, // 0 - UCA, 1 - NUCA
    int core_count,
    int cache_level, // 0 - L2, 1 - L3
    int nuca_bank_count,
    int nuca_obj_func_delay,
    int nuca_obj_func_dynamic_power,
    int nuca_obj_func_leakage_power,
    int nuca_obj_func_area,
    int nuca_obj_func_cycle_time,
    int nuca_dev_func_delay,
    int nuca_dev_func_dynamic_power,
    int nuca_dev_func_leakage_power,
    int nuca_dev_func_area,
    int nuca_dev_func_cycle_time,
    int REPEATERS_IN_HTREE_SEGMENTS_in,//TODO for now only wires with repeaters are supported
    int p_input) {
    g_ip = new InputParameter();
    g_ip->add_ecc_b_ = true;

    g_ip->data_arr_ram_cell_tech_type    = data_arr_ram_cell_tech_flavor_in;
    g_ip->data_arr_peri_global_tech_type = data_arr_peri_global_tech_flavor_in;
    g_ip->tag_arr_ram_cell_tech_type     = tag_arr_ram_cell_tech_flavor_in;
    g_ip->tag_arr_peri_global_tech_type  = tag_arr_peri_global_tech_flavor_in;

    g_ip->ic_proj_type     = interconnect_projection_type_in;
    g_ip->wire_is_mat_type = wire_inside_mat_type_in;
    g_ip->wire_os_mat_type = wire_outside_mat_type_in;
    g_ip->burst_len        = burst_length;
    g_ip->int_prefetch_w   = pre_width;
    g_ip->page_sz_bits     = page_sz;

    g_ip->cache_sz            = cache_size;
    g_ip->line_sz             = line_size;
    g_ip->assoc               = associativity;
    g_ip->nbanks              = banks;
    g_ip->out_w               = output_width;
    g_ip->specific_tag        = specific_tag;
    if (tag_width == 0) {
        g_ip->tag_w = 42;
    } else {
        g_ip->tag_w               = tag_width;
    }

    g_ip->access_mode         = access_mode;
    g_ip->delay_wt = obj_func_delay;
    g_ip->dynamic_power_wt = obj_func_dynamic_power;
    g_ip->leakage_power_wt = obj_func_leakage_power;
    g_ip->area_wt = obj_func_area;
    g_ip->cycle_time_wt    = obj_func_cycle_time;
    g_ip->delay_dev = dev_func_delay;
    g_ip->dynamic_power_dev = dev_func_dynamic_power;
    g_ip->leakage_power_dev = dev_func_leakage_power;
    g_ip->area_dev = dev_func_area;
    g_ip->cycle_time_dev    = dev_func_cycle_time;
    g_ip->ed = ed_ed2_none;

    switch (wt) {
    case (0):
        g_ip->force_wiretype = 0;
        g_ip->wt = Global;
        break;
    case (1):
        g_ip->force_wiretype = 1;
        g_ip->wt = Global;
        break;
    case (2):
        g_ip->force_wiretype = 1;
        g_ip->wt = Global_5;
        break;
    case (3):
        g_ip->force_wiretype = 1;
        g_ip->wt = Global_10;
        break;
    case (4):
        g_ip->force_wiretype = 1;
        g_ip->wt = Global_20;
        break;
    case (5):
        g_ip->force_wiretype = 1;
        g_ip->wt = Global_30;
        break;
    case (6):
        g_ip->force_wiretype = 1;
        g_ip->wt = Low_swing;
        break;
    default:
        cout << "Unknown wire type!\n";
        exit(0);
    }

    g_ip->delay_wt_nuca = nuca_obj_func_delay;
    g_ip->dynamic_power_wt_nuca = nuca_obj_func_dynamic_power;
    g_ip->leakage_power_wt_nuca = nuca_obj_func_leakage_power;
    g_ip->area_wt_nuca = nuca_obj_func_area;
    g_ip->cycle_time_wt_nuca    = nuca_obj_func_cycle_time;
    g_ip->delay_dev_nuca = dev_func_delay;
    g_ip->dynamic_power_dev_nuca = nuca_dev_func_dynamic_power;
    g_ip->leakage_power_dev_nuca = nuca_dev_func_leakage_power;
    g_ip->area_dev_nuca = nuca_dev_func_area;
    g_ip->cycle_time_dev_nuca    = nuca_dev_func_cycle_time;
    g_ip->nuca = is_nuca;
    g_ip->nuca_bank_count = nuca_bank_count;
    if (nuca_bank_count > 0) {
        g_ip->force_nuca_bank = 1;
    }
    g_ip->cores = core_count;
    g_ip->cache_level = cache_level;

    g_ip->temp = temp;

    g_ip->F_sz_nm         = tech_node;
    g_ip->F_sz_um         = tech_node / 1000;
    g_ip->is_main_mem     = (main_mem != 0) ? true : false;
    g_ip->is_cache        = (cache != 0) ? true : false;
    g_ip->rpters_in_htree = (REPEATERS_IN_HTREE_SEGMENTS_in != 0) ? true : false;

    g_ip->num_rw_ports    = rw_ports;
    g_ip->num_rd_ports    = excl_read_ports;
    g_ip->num_wr_ports    = excl_write_ports;
    g_ip->num_se_rd_ports = single_ended_read_ports;
    g_ip->print_detail = 1;
    g_ip->nuca = 0;

    g_ip->wt = Global_5;
    g_ip->force_cache_config = false;
    g_ip->force_wiretype = false;
    g_ip->print_input_args = p_input;


    uca_org_t fin_res;
    fin_res.valid = false;

    if (g_ip->error_checking() == false) exit(0);
    if (g_ip->print_input_args)
        g_ip->display_ip();
    init_tech_params(g_ip->F_sz_um, false);
    Wire winit; // Do not delete this line. It initializes wires.

    if (g_ip->nuca == 1) {
        Nuca n(&g_tp.peri_global);
        n.sim_nuca();
    }
    solve(&fin_res);

    output_UCA(&fin_res);

    delete (g_ip);
    return fin_res;
}

//McPAT's plain interface, please keep !!!
uca_org_t cacti_interface(
    int cache_size,
    int line_size,
    int associativity,
    int rw_ports,
    int excl_read_ports,// para5
    int excl_write_ports,
    int single_ended_read_ports,
    int search_ports,
    int banks,
    double tech_node,//para10
    int output_width,
    int specific_tag,
    int tag_width,
    int access_mode,
    int cache,      //para15
    int main_mem,
    int obj_func_delay,
    int obj_func_dynamic_power,
    int obj_func_leakage_power,
    int obj_func_cycle_time, //para20
    int obj_func_area,
    int dev_func_delay,
    int dev_func_dynamic_power,
    int dev_func_leakage_power,
    int dev_func_area, //para25
    int dev_func_cycle_time,
    int ed_ed2_none, // 0 - ED, 1 - ED^2, 2 - use weight and deviate
    int temp,
    int wt, //0 - default(search across everything), 1 - global, 2 - 5% delay penalty, 3 - 10%, 4 - 20 %, 5 - 30%, 6 - low-swing
    int data_arr_ram_cell_tech_flavor_in,//para30
    int data_arr_peri_global_tech_flavor_in,
    int tag_arr_ram_cell_tech_flavor_in,
    int tag_arr_peri_global_tech_flavor_in,
    int interconnect_projection_type_in,
    int wire_inside_mat_type_in,//para35
    int wire_outside_mat_type_in,
    int REPEATERS_IN_HTREE_SEGMENTS_in,
    int VERTICAL_HTREE_WIRES_OVER_THE_ARRAY_in,
    int BROADCAST_ADDR_DATAIN_OVER_VERTICAL_HTREES_in,
    int PAGE_SIZE_BITS_in,//para40
    int BURST_LENGTH_in,
    int INTERNAL_PREFETCH_WIDTH_in,
    int force_wiretype,
    int wiretype,
    int force_config,//para45
    int ndwl,
    int ndbl,
    int nspd,
    int ndcm,
    int ndsam1,//para50
    int ndsam2,
    int ecc) {
    g_ip = new InputParameter();

    uca_org_t fin_res;
    fin_res.valid = false;

    g_ip->data_arr_ram_cell_tech_type    = data_arr_ram_cell_tech_flavor_in;
    g_ip->data_arr_peri_global_tech_type = data_arr_peri_global_tech_flavor_in;
    g_ip->tag_arr_ram_cell_tech_type     = tag_arr_ram_cell_tech_flavor_in;
    g_ip->tag_arr_peri_global_tech_type  = tag_arr_peri_global_tech_flavor_in;

    g_ip->ic_proj_type     = interconnect_projection_type_in;
    g_ip->wire_is_mat_type = wire_inside_mat_type_in;
    g_ip->wire_os_mat_type = wire_outside_mat_type_in;
    g_ip->burst_len        = BURST_LENGTH_in;
    g_ip->int_prefetch_w   = INTERNAL_PREFETCH_WIDTH_in;
    g_ip->page_sz_bits     = PAGE_SIZE_BITS_in;

    g_ip->cache_sz            = cache_size;
    g_ip->line_sz             = line_size;
    g_ip->assoc               = associativity;
    g_ip->nbanks              = banks;
    g_ip->out_w               = output_width;
    g_ip->specific_tag        = specific_tag;
    if (specific_tag == 0) {
        g_ip->tag_w = 42;
    } else {
        g_ip->tag_w               = tag_width;
    }

    g_ip->access_mode         = access_mode;
    g_ip->delay_wt = obj_func_delay;
    g_ip->dynamic_power_wt = obj_func_dynamic_power;
    g_ip->leakage_power_wt = obj_func_leakage_power;
    g_ip->area_wt = obj_func_area;
    g_ip->cycle_time_wt    = obj_func_cycle_time;
    g_ip->delay_dev = dev_func_delay;
    g_ip->dynamic_power_dev = dev_func_dynamic_power;
    g_ip->leakage_power_dev = dev_func_leakage_power;
    g_ip->area_dev = dev_func_area;
    g_ip->cycle_time_dev    = dev_func_cycle_time;
    g_ip->temp = temp;
    g_ip->ed = ed_ed2_none;

    g_ip->F_sz_nm         = tech_node;
    g_ip->F_sz_um         = tech_node / 1000;
    g_ip->is_main_mem     = (main_mem != 0) ? true : false;
    g_ip->is_cache        = (cache == 1) ? true : false;
    g_ip->pure_ram        = (cache == 0) ? true : false;
    g_ip->pure_cam        = (cache == 2) ? true : false;
    g_ip->rpters_in_htree = (REPEATERS_IN_HTREE_SEGMENTS_in != 0) ? true : false;
    g_ip->ver_htree_wires_over_array = VERTICAL_HTREE_WIRES_OVER_THE_ARRAY_in;
    g_ip->broadcast_addr_din_over_ver_htrees = BROADCAST_ADDR_DATAIN_OVER_VERTICAL_HTREES_in;

    g_ip->num_rw_ports    = rw_ports;
    g_ip->num_rd_ports    = excl_read_ports;
    g_ip->num_wr_ports    = excl_write_ports;
    g_ip->num_se_rd_ports = single_ended_read_ports;
    g_ip->num_search_ports = search_ports;

    g_ip->print_detail = 1;
    g_ip->nuca = 0;

    if (force_wiretype == 0) {
        g_ip->wt = Global;
        g_ip->force_wiretype = false;
    } else {
        g_ip->force_wiretype = true;
        if (wiretype == 10) {
            g_ip->wt = Global_10;
        }
        if (wiretype == 20) {
            g_ip->wt = Global_20;
        }
        if (wiretype == 30) {
            g_ip->wt = Global_30;
        }
        if (wiretype == 5) {
            g_ip->wt = Global_5;
        }
        if (wiretype == 0) {
            g_ip->wt = Low_swing;
        }
    }
    //g_ip->wt = Global_5;
    if (force_config == 0) {
        g_ip->force_cache_config = false;
    } else {
        g_ip->force_cache_config = true;
        g_ip->ndbl = ndbl;
        g_ip->ndwl = ndwl;
        g_ip->nspd = nspd;
        g_ip->ndcm = ndcm;
        g_ip->ndsam1 = ndsam1;
        g_ip->ndsam2 = ndsam2;


    }

    if (ecc == 0) {
        g_ip->add_ecc_b_ = false;
    } else {
        g_ip->add_ecc_b_ = true;
    }


    if (!g_ip->error_checking())
        exit(0);

    init_tech_params(g_ip->F_sz_um, false);
    Wire winit; // Do not delete this line. It initializes wires.

    g_ip->display_ip();
    solve(&fin_res);
    output_UCA(&fin_res);
    output_data_csv(fin_res);
    delete (g_ip);

    return fin_res;
}



bool InputParameter::error_checking(string name) {
    int  A;
    bool seq_access  = false;
    fast_access = true;

    switch (access_mode) {
    case 0:
        seq_access  = false;
        fast_access = false;
        break;
    case 1:
        seq_access  = true;
        fast_access = false;
        break;
    case 2:
        seq_access  = false;
        fast_access = true;
        break;
    }

    if (is_main_mem) {
        if (ic_proj_type == 0) {
            cerr << name
                 << ": DRAM model supports only conservative interconnect "
                 << "projection but is set to aggressive!\n\n";
            return false;
        }
    }


    uint32_t B = line_sz;

    if (B < 1) {
        cerr << name << ": Block size must be >= 1, but is set to " << B
             << endl;
        return false;
    } else if (B*8 < out_w) {
        cerr << name << ": Block size must be at least " << out_w / 8
             << ", but is set to " << B << endl;
        return false;
    }

    if (F_sz_um <= 0) {
        cerr << name << ": Feature size must be > 0, but is set to "
             << F_sz_um << endl;
        return false;
    } else if (F_sz_um > 0.091) {
        cerr << name << ": Feature size must be <= 90 nm, but is set to "
             << F_sz_um << endl;
        return false;
    }


    uint32_t RWP  = num_rw_ports;
    uint32_t ERP  = num_rd_ports;
    uint32_t EWP  = num_wr_ports;
    uint32_t NSER = num_se_rd_ports;
    uint32_t SCHP = num_search_ports;

//TODO: revisit this. This is an important feature. Sheng thought this should be used
//  // If multiple banks and multiple ports are specified, then if number of ports is less than or equal to
//  // the number of banks, we assume that the multiple ports are implemented via the multiple banks.
//  // In such a case we assume that each bank has 1 RWP port.
//  if ((RWP + ERP + EWP) <= nbanks && nbanks>1)
//  {
//    RWP  = 1;
//    ERP  = 0;
//    EWP  = 0;
//    NSER = 0;
//  }
//  else if ((RWP < 0) || (EWP < 0) || (ERP < 0))
//  {
//    cerr << "Ports must >=0" << endl;
//    return false;
//  }
//  else if (RWP > 2)
//  {
//    cerr << "Maximum of 2 read/write ports" << endl;
//    return false;
//  }
//  else if ((RWP+ERP+EWP) < 1)
    // Changed to new implementation:
    // The number of ports specified at input is per bank
    if ((RWP + ERP + EWP) < 1) {
        cerr << name << ": Must have at least one port" << endl;
        return false;
    }

    if (is_pow2(nbanks) == false) {
        cerr << name << ": Number of subbanks should be greater than or "
             << "equal to 1 and should be a power of 2, but is set to "
             << nbanks << endl;
        return false;
    }

    int C = cache_sz / nbanks;
    if (C < 64) {
        cerr << name << ": Cache size must be >=64, but is set to " << C
             << endl;
        return false;
    }

//TODO: revisit this
//   if (pure_ram==true && assoc!=1)
//    {
//  	  cerr << "Pure RAM must have assoc as 1" << endl;
//  	  return false;
//    }

    //fully assoc and cam check
    if (is_cache && assoc == 0)
        fully_assoc = true;
    else
        fully_assoc = false;

    if (pure_cam == true && assoc != 0) {
        cerr << name
             << ": Pure CAM must have associativity as 0, but is set to"
             << assoc << endl;
        return false;
    }

    if (assoc == 0 && (pure_cam == false && is_cache == false)) {
        cerr << name
             << ": Only CAM or Fully associative cache can have associativity "
             << "as 0" << endl;
        return false;
    }

    if ((fully_assoc == true || pure_cam == true)
        && (data_arr_ram_cell_tech_type != tag_arr_ram_cell_tech_type
            || data_arr_peri_global_tech_type != tag_arr_peri_global_tech_type)) {
        cerr << name
             << ": CAM and fully associative cache must have same device type "
             << "for both data and tag array" << endl;
        cerr << "\tData array RAM cell = " << data_arr_ram_cell_tech_type
             << ", Tag array RAM cell = " << tag_arr_ram_cell_tech_type << endl
             << "\tData array peripheral = " << data_arr_peri_global_tech_type
             << ", Tag array peripheral = " << tag_arr_peri_global_tech_type
             << endl;
        return false;
    }

    if ((fully_assoc == true || pure_cam == true)
            &&  (data_arr_ram_cell_tech_type == lp_dram ||
                 data_arr_ram_cell_tech_type == comm_dram)) {
        cerr << name << ": DRAM based CAM and fully associative cache are not "
             << "supported" << endl;
        return false;
    }

    if ((fully_assoc == true || pure_cam == true)
            &&  (is_main_mem == true)) {
        cerr << name
             << ": CAM and fully associative cache cannot be as main memory"
             << endl;
        return false;
    }

    if ((fully_assoc || pure_cam) && SCHP < 1) {
        cerr << name
             << ": CAM and fully associative must have at least 1 search port,"
             << " but are set to " << SCHP << endl;
        return false;
    }

    if (RWP == 0 && ERP == 0 && SCHP > 0 && ((fully_assoc || pure_cam))) {
        ERP = SCHP;
    }

//    if ((!(fully_assoc || pure_cam)) && SCHP>=1)
//    {
//	  cerr << "None CAM and fully associative cannot have search ports" << endl;
//	  return false;
//    }

    if (assoc == 0) {
        A = C / B;
        //fully_assoc = true;
    } else {
        if (assoc == 1) {
            A = 1;
            //fully_assoc = false;
        } else {
            //fully_assoc = false;
            A = assoc;
            if (is_pow2(A) == false) {
                cerr << name
                     << ": Associativity must be a power of 2, but is set to "
                     << A << endl;
                return false;
            }
        }
    }

    if (C / (B*A) <= 1 && assoc != 0) {
        cerr << name << ": Number of sets (" << (C / (B * A))
             << ") is too small: " << endl;
        cerr << " Need to either increase cache size, or decrease "
             << "associativity or block size" << endl;
        cerr << " (or use fully associative cache)" << endl;
        return false;
    }

    block_sz = B;

    /*dt: testing sequential access mode*/
    if (seq_access) {
        tag_assoc  = A;
        data_assoc = 1;
        is_seq_acc = true;
    } else {
        tag_assoc  = A;
        data_assoc = A;
        is_seq_acc = false;
    }

    if (assoc == 0) {
        data_assoc = 1;
    }
    num_rw_ports     = RWP;
    num_rd_ports     = ERP;
    num_wr_ports     = EWP;
    num_se_rd_ports  = NSER;
    if (!(fully_assoc || pure_cam))
        num_search_ports = 0;
    nsets            = C / (B * A);

    if (temp < 300 || temp > 400 || temp % 10 != 0) {
        cerr << name << ": " << temp
             << " Temperature must be between 300 and 400 Kelvin and multiple "
             << "of 10." << endl;
        return false;
    }

    if (nsets < 1) {
        cerr << name << ": Less than one set..." << endl;
        return false;
    }

    return true;
}



void output_data_csv(const uca_org_t & fin_res) {
    //TODO: the csv output should remain
    fstream file("out.csv", ios::in);
    bool    print_index = file.fail();
    file.close();

    file.open("out.csv", ios::out | ios::app);
    if (file.fail() == true) {
        cerr << "File out.csv could not be opened successfully" << endl;
    } else {
        if (print_index == true) {
            file << "Tech node (nm), ";
            file << "Capacity (bytes), ";
            file << "Number of banks, ";
            file << "Associativity, ";
            file << "Output width (bits), ";
            file << "Access time (ns), ";
            file << "Random cycle time (ns), ";
            file << "Dynamic search energy (nJ), ";
            file << "Dynamic read energy (nJ), ";
            file << "Dynamic write energy (nJ), ";
            file << "Standby leakage per bank(mW), ";
            file << "Area (mm2), ";
            file << "Ndwl, ";
            file << "Ndbl, ";
            file << "Nspd, ";
            file << "Ndcm, ";
            file << "Ndsam_level_1, ";
            file << "Ndsam_level_2, ";
            file << "Data arrary area efficiency %, ";
            file << "Ntwl, ";
            file << "Ntbl, ";
            file << "Ntspd, ";
            file << "Ntcm, ";
            file << "Ntsam_level_1, ";
            file << "Ntsam_level_2, ";
            file << "Tag arrary area efficiency %, ";

//      file << "Resistance per unit micron (ohm-micron), ";
//      file << "Capacitance per unit micron (fF per micron), ";
//      file << "Unit-length wire delay (ps), ";
//      file << "FO4 delay (ps), ";
//      file << "delay route to bank (including crossb delay) (ps), ";
//      file << "Crossbar delay (ps), ";
//      file << "Dyn read energy per access from closed page (nJ), ";
//      file << "Dyn read energy per access from open page (nJ), ";
//      file << "Leak power of an subbank with page closed (mW), ";
//      file << "Leak power of a subbank with page  open (mW), ";
//      file << "Leak power of request and reply networks (mW), ";
//      file << "Number of subbanks, ";
//      file << "Page size in bits, ";
//      file << "Activate power, ";
//      file << "Read power, ";
//      file << "Write power, ";
//      file << "Precharge power, ";
//      file << "tRCD, ";
//      file << "CAS latency, ";
//      file << "Precharge delay, ";
//      file << "Perc dyn energy bitlines, ";
//      file << "perc dyn energy wordlines, ";
//      file << "perc dyn energy outside mat, ";
//      file << "Area opt (perc), ";
//      file << "Delay opt (perc), ";
//      file << "Repeater opt (perc), ";
//      file << "Aspect ratio";
            file << endl;
        }
        file << g_ip->F_sz_nm << ", ";
        file << g_ip->cache_sz << ", ";
        file << g_ip->nbanks << ", ";
        file << g_ip->tag_assoc << ", ";
        file << g_ip->out_w << ", ";
        file << fin_res.access_time*1e+9 << ", ";
        file << fin_res.cycle_time*1e+9 << ", ";
//    file << fin_res.data_array2->multisubbank_interleave_cycle_time*1e+9 << ", ";
//    file << fin_res.data_array2->delay_request_network*1e+9 << ", ";
//    file << fin_res.data_array2->delay_inside_mat*1e+9 <<  ", ";
//    file << fin_res.data_array2.delay_reply_network*1e+9 << ", ";

//    if (!(g_ip->fully_assoc || g_ip->pure_cam || g_ip->pure_ram))
//        {
//    	  file << fin_res.tag_array2->access_time*1e+9 << ", ";
//        }
//    else
//    {
//    	file << 0 << ", ";
//    }
//    file << fin_res.data_array2->access_time*1e+9 << ", ";
//    file << fin_res.data_array2->dram_refresh_period*1e+6 << ", ";
//    file << fin_res.data_array2->dram_array_availability <<  ", ";
        if (g_ip->fully_assoc || g_ip->pure_cam) {
            file << fin_res.power.searchOp.dynamic*1e+9 << ", ";
        } else {
            file << "N/A" << ", ";
        }
        file << fin_res.power.readOp.dynamic*1e+9 << ", ";
        file << fin_res.power.writeOp.dynamic*1e+9 << ", ";
//    if (!(g_ip->fully_assoc || g_ip->pure_cam || g_ip->pure_ram))
//        {
//        	file << fin_res.tag_array2->power.readOp.dynamic*1e+9 << ", ";
//        }
//        	else
//        {
//        		file << "NA" << ", ";
//        }
//    file << fin_res.data_array2->power.readOp.dynamic*1e+9 << ", ";
//    if (g_ip->fully_assoc || g_ip->pure_cam)
//        {
//    	    file << fin_res.power.searchOp.dynamic*1000/fin_res.cycle_time << ", ";
//        }
//        	else
//        {
//        	file << fin_res.power.readOp.dynamic*1000/fin_res.cycle_time << ", ";
//        }

    file <<( fin_res.power.readOp.leakage + fin_res.power.readOp.gate_leakage )*1000 << ", ";
//    file << fin_res.leak_power_with_sleep_transistors_in_mats*1000 << ", ";
//    file << fin_res.data_array.refresh_power / fin_res.data_array.total_power.readOp.leakage << ", ";
        file << fin_res.area*1e-6 << ", ";

        file << fin_res.data_array2->Ndwl << ", ";
        file << fin_res.data_array2->Ndbl << ", ";
        file << fin_res.data_array2->Nspd << ", ";
        file << fin_res.data_array2->deg_bl_muxing << ", ";
        file << fin_res.data_array2->Ndsam_lev_1 << ", ";
        file << fin_res.data_array2->Ndsam_lev_2 << ", ";
        file << fin_res.data_array2->area_efficiency << ", ";
        if (!(g_ip->fully_assoc || g_ip->pure_cam || g_ip->pure_ram)) {
            file << fin_res.tag_array2->Ndwl << ", ";
            file << fin_res.tag_array2->Ndbl << ", ";
            file << fin_res.tag_array2->Nspd << ", ";
            file << fin_res.tag_array2->deg_bl_muxing << ", ";
            file << fin_res.tag_array2->Ndsam_lev_1 << ", ";
            file << fin_res.tag_array2->Ndsam_lev_2 << ", ";
            file << fin_res.tag_array2->area_efficiency << ", ";
        } else {
        file << "N/A" << ", ";
        file << "N/A"<< ", ";
        file << "N/A" << ", ";
        file << "N/A" << ", ";
        file << "N/A" << ", ";
        file << "N/A" << ", ";
        file << "N/A" << ", ";
    }

//    file << g_tp.wire_inside_mat.R_per_um << ", ";
//    file << g_tp.wire_inside_mat.C_per_um / 1e-15 << ", ";
//    file << g_tp.unit_len_wire_del / 1e-12 << ", ";
//    file << g_tp.FO4 / 1e-12 << ", ";
//    file << fin_res.data_array.delay_route_to_bank / 1e-9 << ", ";
//    file << fin_res.data_array.delay_crossbar / 1e-9 << ", ";
//    file << fin_res.data_array.dyn_read_energy_from_closed_page / 1e-9 << ", ";
//    file << fin_res.data_array.dyn_read_energy_from_open_page / 1e-9 << ", ";
//    file << fin_res.data_array.leak_power_subbank_closed_page / 1e-3 << ", ";
//    file << fin_res.data_array.leak_power_subbank_open_page / 1e-3 << ", ";
//    file << fin_res.data_array.leak_power_request_and_reply_networks / 1e-3 << ", ";
//    file << fin_res.data_array.number_subbanks << ", " ;
//    file << fin_res.data_array.page_size_in_bits << ", " ;
//    file << fin_res.data_array.activate_energy * 1e9 << ", " ;
//    file << fin_res.data_array.read_energy * 1e9 << ", " ;
//    file << fin_res.data_array.write_energy * 1e9 << ", " ;
//    file << fin_res.data_array.precharge_energy * 1e9 << ", " ;
//    file << fin_res.data_array.trcd * 1e9 << ", " ;
//    file << fin_res.data_array.cas_latency * 1e9 << ", " ;
//    file << fin_res.data_array.precharge_delay * 1e9 << ", " ;
//    file << fin_res.data_array.all_banks_height / fin_res.data_array.all_banks_width;
        file<<endl;
    }
    file.close();
}



void output_UCA(uca_org_t *fr) {
    //    if (NUCA)
    if (0) {
        cout << "\n\n Detailed Bank Stats:\n";
        cout << "    Bank Size (bytes): %d\n" <<
             (int) (g_ip->cache_sz);
    } else {
        if (g_ip->data_arr_ram_cell_tech_type == 3) {
            cout << "\n---------- CACTI version 6.5, Uniform Cache Access " <<
                 "Logic Process Based DRAM Model ----------\n";
        } else if (g_ip->data_arr_ram_cell_tech_type == 4) {
            cout << "\n---------- CACTI version 6.5, Uniform" <<
                 "Cache Access Commodity DRAM Model ----------\n";
        } else {
            cout << "\n---------- CACTI version 6.5, Uniform Cache Access "
                 "SRAM Model ----------\n";
        }
        cout << "\nCache Parameters:\n";
        cout << "    Total cache size (bytes): " <<
             (int) (g_ip->cache_sz) << endl;
    }

    cout << "    Number of banks: " << (int) g_ip->nbanks << endl;
    if (g_ip->fully_assoc || g_ip->pure_cam)
        cout << "    Associativity: fully associative\n";
    else {
        if (g_ip->tag_assoc == 1)
            cout << "    Associativity: direct mapped\n";
        else
            cout << "    Associativity: " <<
                 g_ip->tag_assoc << endl;
    }


    cout << "    Block size (bytes): " << g_ip->line_sz << endl;
    cout << "    Read/write Ports: " <<
         g_ip->num_rw_ports << endl;
    cout << "    Read ports: " <<
         g_ip->num_rd_ports << endl;
    cout << "    Write ports: " <<
         g_ip->num_wr_ports << endl;
    if (g_ip->fully_assoc || g_ip->pure_cam)
        cout << "    search ports: " <<
             g_ip->num_search_ports << endl;
    cout << "    Technology size (nm): " <<
         g_ip->F_sz_nm << endl << endl;

    cout << "    Access time (ns): " << fr->access_time*1e9 << endl;
    cout << "    Cycle time (ns):  " << fr->cycle_time*1e9 << endl;
    if (g_ip->data_arr_ram_cell_tech_type >= 4) {
        cout << "    Precharge Delay (ns): " << fr->data_array2->precharge_delay*1e9 << endl;
        cout << "    Activate Energy (nJ): " << fr->data_array2->activate_energy*1e9 << endl;
        cout << "    Read Energy (nJ): " << fr->data_array2->read_energy*1e9 << endl;
        cout << "    Write Energy (nJ): " << fr->data_array2->write_energy*1e9 << endl;
        cout << "    Precharge Energy (nJ): " << fr->data_array2->precharge_energy*1e9 << endl;
        cout << "    Leakage Power Closed Page (mW): " << fr->data_array2->leak_power_subbank_closed_page*1e3 << endl;
        cout << "    Leakage Power Open Page (mW): " << fr->data_array2->leak_power_subbank_open_page*1e3 << endl;
        cout << "    Leakage Power I/O (mW): " << fr->data_array2->leak_power_request_and_reply_networks*1e3 << endl;
        cout << "    Refresh power (mW): " <<
             fr->data_array2->refresh_power*1e3 << endl;
    } else {
        if ((g_ip->fully_assoc || g_ip->pure_cam)) {
            cout << "    Total dynamic associative search energy per access (nJ): " <<
                 fr->power.searchOp.dynamic*1e9 << endl;
//		  cout << "    Total dynamic read energy per access (nJ): " <<
//		  fr->power.readOp.dynamic*1e9 << endl;
//		  cout << "    Total dynamic write energy per access (nJ): " <<
//		  fr->power.writeOp.dynamic*1e9 << endl;
        }
//	  else
//	  {
        cout << "    Total dynamic read energy per access (nJ): " <<
             fr->power.readOp.dynamic*1e9 << endl;
        cout << "    Total dynamic write energy per access (nJ): " <<
             fr->power.writeOp.dynamic*1e9 << endl;
//	  }
        cout << "    Total leakage power of a bank"
             " (mW): " << fr->power.readOp.leakage*1e3 << endl;
        cout << "    Total gate leakage power of a bank"
             " (mW): " << fr->power.readOp.gate_leakage*1e3 << endl;
    }

    if (g_ip->data_arr_ram_cell_tech_type == 3 || g_ip->data_arr_ram_cell_tech_type == 4) {
    }
    cout <<  "    Cache height x width (mm): " <<
         fr->cache_ht*1e-3 << " x " << fr->cache_len*1e-3 << endl << endl;


    cout << "    Best Ndwl : " << fr->data_array2->Ndwl << endl;
    cout << "    Best Ndbl : " << fr->data_array2->Ndbl << endl;
    cout << "    Best Nspd : " << fr->data_array2->Nspd << endl;
    cout << "    Best Ndcm : " << fr->data_array2->deg_bl_muxing << endl;
    cout << "    Best Ndsam L1 : " << fr->data_array2->Ndsam_lev_1 << endl;
    cout << "    Best Ndsam L2 : " << fr->data_array2->Ndsam_lev_2 << endl << endl;

    if ((!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) &&
        !g_ip->is_main_mem) {
        cout << "    Best Ntwl : " << fr->tag_array2->Ndwl << endl;
        cout << "    Best Ntbl : " << fr->tag_array2->Ndbl << endl;
        cout << "    Best Ntspd : " << fr->tag_array2->Nspd << endl;
        cout << "    Best Ntcm : " << fr->tag_array2->deg_bl_muxing << endl;
        cout << "    Best Ntsam L1 : " << fr->tag_array2->Ndsam_lev_1 << endl;
        cout << "    Best Ntsam L2 : " << fr->tag_array2->Ndsam_lev_2 << endl;
    }

    switch (fr->data_array2->wt) {
    case (0):
        cout <<  "    Data array, H-tree wire type: Delay optimized global wires\n";
        break;
    case (1):
        cout <<  "    Data array, H-tree wire type: Global wires with 5\% delay penalty\n";
        break;
    case (2):
        cout <<  "    Data array, H-tree wire type: Global wires with 10\% delay penalty\n";
        break;
    case (3):
        cout <<  "    Data array, H-tree wire type: Global wires with 20\% delay penalty\n";
        break;
    case (4):
        cout <<  "    Data array, H-tree wire type: Global wires with 30\% delay penalty\n";
        break;
    case (5):
        cout <<  "    Data array, wire type: Low swing wires\n";
        break;
    default:
        cout << "ERROR - Unknown wire type " << (int) fr->data_array2->wt << endl;
        exit(0);
    }

    if (!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) {
        switch (fr->tag_array2->wt) {
        case (0):
            cout <<  "    Tag array, H-tree wire type: Delay optimized global wires\n";
            break;
        case (1):
            cout <<  "    Tag array, H-tree wire type: Global wires with 5\% delay penalty\n";
            break;
        case (2):
            cout <<  "    Tag array, H-tree wire type: Global wires with 10\% delay penalty\n";
            break;
        case (3):
            cout <<  "    Tag array, H-tree wire type: Global wires with 20\% delay penalty\n";
            break;
        case (4):
            cout <<  "    Tag array, H-tree wire type: Global wires with 30\% delay penalty\n";
            break;
        case (5):
            cout <<  "    Tag array, wire type: Low swing wires\n";
            break;
        default:
            cout << "ERROR - Unknown wire type " << (int) fr->tag_array2->wt << endl;
            exit(-1);
        }
    }

    if (g_ip->print_detail) {
        /* Delay stats */
        /* data array stats */
        cout << endl << "Time Components:" << endl << endl;

        cout << "  Data side (with Output driver) (ns): " <<
             fr->data_array2->access_time / 1e-9 << endl;

        cout <<  "\tH-tree input delay (ns): " <<
             fr->data_array2->delay_route_to_bank * 1e9 +
             fr->data_array2->delay_input_htree * 1e9 << endl;

        if (!(g_ip->pure_cam || g_ip->fully_assoc)) {
            cout <<  "\tDecoder + wordline delay (ns): " <<
                 fr->data_array2->delay_row_predecode_driver_and_block * 1e9 +
                 fr->data_array2->delay_row_decoder * 1e9 << endl;
        } else {
            cout <<  "\tCAM search delay (ns): " <<
                 fr->data_array2->delay_matchlines * 1e9 << endl;
        }

        cout <<  "\tBitline delay (ns): " <<
             fr->data_array2->delay_bitlines / 1e-9 << endl;

        cout <<  "\tSense Amplifier delay (ns): " <<
             fr->data_array2->delay_sense_amp * 1e9 << endl;


        cout <<  "\tH-tree output delay (ns): " <<
             fr->data_array2->delay_subarray_output_driver * 1e9 +
             fr->data_array2->delay_dout_htree * 1e9 << endl;

        if ((!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) &&
            !g_ip->is_main_mem) {
            /* tag array stats */
            cout << endl << "  Tag side (with Output driver) (ns): " <<
                 fr->tag_array2->access_time / 1e-9 << endl;

            cout <<  "\tH-tree input delay (ns): " <<
                 fr->tag_array2->delay_route_to_bank * 1e9 +
                 fr->tag_array2->delay_input_htree * 1e9 << endl;

            cout <<  "\tDecoder + wordline delay (ns): " <<
                 fr->tag_array2->delay_row_predecode_driver_and_block * 1e9 +
                 fr->tag_array2->delay_row_decoder * 1e9 << endl;

            cout <<  "\tBitline delay (ns): " <<
                 fr->tag_array2->delay_bitlines / 1e-9 << endl;

            cout <<  "\tSense Amplifier delay (ns): " <<
                 fr->tag_array2->delay_sense_amp * 1e9 << endl;

            cout <<  "\tComparator delay (ns): " <<
                 fr->tag_array2->delay_comparator * 1e9 << endl;

            cout <<  "\tH-tree output delay (ns): " <<
                 fr->tag_array2->delay_subarray_output_driver * 1e9 +
                 fr->tag_array2->delay_dout_htree * 1e9 << endl;
        }



        /* Energy/Power stats */
        cout << endl << endl << "Power Components:" << endl << endl;

        if (!(g_ip->pure_cam || g_ip->fully_assoc)) {
            cout << "  Data array: Total dynamic read energy/access  (nJ): " <<
                 fr->data_array2->power.readOp.dynamic * 1e9 << endl;
            cout << "\tTotal leakage read/write power of a bank (mW): " <<
                 fr->data_array2->power.readOp.leakage * 1e3 << endl;

            cout << "\tTotal energy in H-tree (that includes both "
                 "address and data transfer) (nJ): " <<
                 (fr->data_array2->power_addr_input_htree.readOp.dynamic +
                  fr->data_array2->power_data_output_htree.readOp.dynamic +
                  fr->data_array2->power_routing_to_bank.readOp.dynamic) * 1e9 << endl;

            cout << "\tTotal leakage power in H-tree (that includes both "
                "address and data network) ((mW)): " <<
                (fr->data_array2->power_addr_input_htree.readOp.leakage +
                 fr->data_array2->power_data_output_htree.readOp.leakage +
                 fr->data_array2->power_routing_to_bank.readOp.leakage) * 1e3
                 << endl;

            cout << "\tTotal gate leakage power in H-tree (that includes both "
                "address and data network) ((mW)): " <<
                (fr->data_array2->power_addr_input_htree.readOp.gate_leakage +
                 fr->data_array2->power_data_output_htree.readOp.gate_leakage +
                 fr->data_array2->power_routing_to_bank.readOp.gate_leakage) *
                1e3 << endl;

            cout << "\tOutput Htree inside bank Energy (nJ): " <<
                 fr->data_array2->power_data_output_htree.readOp.dynamic * 1e9 << endl;
            cout <<  "\tDecoder (nJ): " <<
                 fr->data_array2->power_row_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_row_predecoder_blocks.readOp.dynamic * 1e9 << endl;
            cout <<  "\tWordline (nJ): " <<
                 fr->data_array2->power_row_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitline mux & associated drivers (nJ): " <<
                 fr->data_array2->power_bit_mux_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amp mux & associated drivers (nJ): " <<
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_decoders.readOp.dynamic * 1e9  +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_decoders.readOp.dynamic * 1e9 << endl;

            cout <<  "\tBitlines precharge and equalization circuit (nJ): " <<
                 fr->data_array2->power_prechg_eq_drivers.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitlines (nJ): " <<
                 fr->data_array2->power_bitlines.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amplifier energy (nJ): " <<
                 fr->data_array2->power_sense_amps.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->data_array2->power_output_drivers_at_subarray.readOp.dynamic * 1e9 << endl;
        }

        else if (g_ip->pure_cam) {

            cout << "  CAM array:" << endl;
            cout << "  Total dynamic associative search energy/access  (nJ): " <<
                 fr->data_array2->power.searchOp.dynamic * 1e9 << endl;
            cout << "\tTotal energy in H-tree (that includes both "
                 "match key and data transfer) (nJ): " <<
                 (fr->data_array2->power_htree_in_search.searchOp.dynamic +
                  fr->data_array2->power_htree_out_search.searchOp.dynamic +
                  fr->data_array2->power_routing_to_bank.searchOp.dynamic) * 1e9 << endl;
            cout << "\tKeyword input and result output Htrees inside bank Energy (nJ): " <<
                 (fr->data_array2->power_htree_in_search.searchOp.dynamic +
                  fr->data_array2->power_htree_out_search.searchOp.dynamic) * 1e9 << endl;
            cout <<  "\tSearchlines (nJ): " <<
                 fr->data_array2->power_searchline.searchOp.dynamic * 1e9 +
                 fr->data_array2->power_searchline_precharge.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tMatchlines  (nJ): " <<
                 fr->data_array2->power_matchlines.searchOp.dynamic * 1e9 +
                 fr->data_array2->power_matchline_precharge.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->data_array2->power_output_drivers_at_subarray.searchOp.dynamic * 1e9 << endl;


            cout << endl << "  Total dynamic read energy/access  (nJ): " <<
                 fr->data_array2->power.readOp.dynamic * 1e9 << endl;
            cout << "\tTotal energy in H-tree (that includes both "
                 "address and data transfer) (nJ): " <<
                 (fr->data_array2->power_addr_input_htree.readOp.dynamic +
                  fr->data_array2->power_data_output_htree.readOp.dynamic +
                  fr->data_array2->power_routing_to_bank.readOp.dynamic) * 1e9 << endl;
            cout << "\tOutput Htree inside bank Energy (nJ): " <<
                 fr->data_array2->power_data_output_htree.readOp.dynamic * 1e9 << endl;
            cout <<  "\tDecoder (nJ): " <<
                 fr->data_array2->power_row_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_row_predecoder_blocks.readOp.dynamic * 1e9 << endl;
            cout <<  "\tWordline (nJ): " <<
                 fr->data_array2->power_row_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitline mux & associated drivers (nJ): " <<
                 fr->data_array2->power_bit_mux_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amp mux & associated drivers (nJ): " <<
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_decoders.readOp.dynamic * 1e9  +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitlines (nJ): " <<
                 fr->data_array2->power_bitlines.readOp.dynamic * 1e9 +
                 fr->data_array2->power_prechg_eq_drivers.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amplifier energy (nJ): " <<
                 fr->data_array2->power_sense_amps.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->data_array2->power_output_drivers_at_subarray.readOp.dynamic * 1e9 << endl;

            cout << endl << "  Total leakage power of a bank (mW): " <<
                 fr->data_array2->power.readOp.leakage * 1e3 << endl;
        } else {
            cout << "  Fully associative array:" << endl;
            cout << "  Total dynamic associative search energy/access  (nJ): " <<
                 fr->data_array2->power.searchOp.dynamic * 1e9 << endl;
            cout << "\tTotal energy in H-tree (that includes both "
                 "match key and data transfer) (nJ): " <<
                 (fr->data_array2->power_htree_in_search.searchOp.dynamic +
                  fr->data_array2->power_htree_out_search.searchOp.dynamic +
                  fr->data_array2->power_routing_to_bank.searchOp.dynamic) * 1e9 << endl;
            cout << "\tKeyword input and result output Htrees inside bank Energy (nJ): " <<
                 (fr->data_array2->power_htree_in_search.searchOp.dynamic +
                  fr->data_array2->power_htree_out_search.searchOp.dynamic) * 1e9 << endl;
            cout <<  "\tSearchlines (nJ): " <<
                 fr->data_array2->power_searchline.searchOp.dynamic * 1e9 +
                 fr->data_array2->power_searchline_precharge.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tMatchlines  (nJ): " <<
                 fr->data_array2->power_matchlines.searchOp.dynamic * 1e9 +
                 fr->data_array2->power_matchline_precharge.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tData portion wordline (nJ): " <<
                 fr->data_array2->power_matchline_to_wordline_drv.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tData Bitlines (nJ): " <<
                 fr->data_array2->power_bitlines.searchOp.dynamic * 1e9 +
                 fr->data_array2->power_prechg_eq_drivers.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amplifier energy (nJ): " <<
                 fr->data_array2->power_sense_amps.searchOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->data_array2->power_output_drivers_at_subarray.searchOp.dynamic * 1e9 << endl;


            cout << endl << "  Total dynamic read energy/access  (nJ): " <<
                 fr->data_array2->power.readOp.dynamic * 1e9 << endl;
            cout << "\tTotal energy in H-tree (that includes both "
                 "address and data transfer) (nJ): " <<
                 (fr->data_array2->power_addr_input_htree.readOp.dynamic +
                  fr->data_array2->power_data_output_htree.readOp.dynamic +
                  fr->data_array2->power_routing_to_bank.readOp.dynamic) * 1e9 << endl;
            cout << "\tOutput Htree inside bank Energy (nJ): " <<
                 fr->data_array2->power_data_output_htree.readOp.dynamic * 1e9 << endl;
            cout <<  "\tDecoder (nJ): " <<
                 fr->data_array2->power_row_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_row_predecoder_blocks.readOp.dynamic * 1e9 << endl;
            cout <<  "\tWordline (nJ): " <<
                 fr->data_array2->power_row_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitline mux & associated drivers (nJ): " <<
                 fr->data_array2->power_bit_mux_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_bit_mux_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amp mux & associated drivers (nJ): " <<
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_1_decoders.readOp.dynamic * 1e9  +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->data_array2->power_senseamp_mux_lev_2_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitlines (nJ): " <<
                 fr->data_array2->power_bitlines.readOp.dynamic * 1e9 +
                 fr->data_array2->power_prechg_eq_drivers.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amplifier energy (nJ): " <<
                 fr->data_array2->power_sense_amps.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->data_array2->power_output_drivers_at_subarray.readOp.dynamic * 1e9 << endl;

            cout << endl << "  Total leakage power of a bank (mW): " <<
                 fr->data_array2->power.readOp.leakage * 1e3 << endl;
        }


        if ((!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) &&
            !g_ip->is_main_mem) {
            cout << endl << "  Tag array:  Total dynamic read energy/access (nJ): " <<
                 fr->tag_array2->power.readOp.dynamic * 1e9 << endl;
            cout << "\tTotal leakage read/write power of a bank (mW): " <<
                 fr->tag_array2->power.readOp.leakage * 1e3 << endl;
            cout << "\tTotal energy in H-tree (that includes both "
                 "address and data transfer) (nJ): " <<
                 (fr->tag_array2->power_addr_input_htree.readOp.dynamic +
                  fr->tag_array2->power_data_output_htree.readOp.dynamic +
                  fr->tag_array2->power_routing_to_bank.readOp.dynamic) * 1e9 << endl;

            cout << "\tTotal leakage power in H-tree (that includes both "
                "address and data network) ((mW)): " <<
                (fr->tag_array2->power_addr_input_htree.readOp.leakage +
                 fr->tag_array2->power_data_output_htree.readOp.leakage +
                 fr->tag_array2->power_routing_to_bank.readOp.leakage) * 1e3
                 << endl;

            cout << "\tTotal gate leakage power in H-tree (that includes both "
                "address and data network) ((mW)): " <<
                (fr->tag_array2->power_addr_input_htree.readOp.gate_leakage +
                 fr->tag_array2->power_data_output_htree.readOp.gate_leakage +
                 fr->tag_array2->power_routing_to_bank.readOp.gate_leakage) *
                1e3 << endl;

            cout << "\tOutput Htree inside a bank Energy (nJ): " <<
                 fr->tag_array2->power_data_output_htree.readOp.dynamic * 1e9 << endl;
            cout <<  "\tDecoder (nJ): " <<
                 fr->tag_array2->power_row_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_row_predecoder_blocks.readOp.dynamic * 1e9 << endl;
            cout <<  "\tWordline (nJ): " <<
                 fr->tag_array2->power_row_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitline mux & associated drivers (nJ): " <<
                 fr->tag_array2->power_bit_mux_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_bit_mux_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_bit_mux_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amp mux & associated drivers (nJ): " <<
                 fr->tag_array2->power_senseamp_mux_lev_1_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_senseamp_mux_lev_1_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_senseamp_mux_lev_1_decoders.readOp.dynamic * 1e9  +
                 fr->tag_array2->power_senseamp_mux_lev_2_predecoder_drivers.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_senseamp_mux_lev_2_predecoder_blocks.readOp.dynamic * 1e9 +
                 fr->tag_array2->power_senseamp_mux_lev_2_decoders.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitlines precharge and equalization circuit (nJ): " <<
                 fr->tag_array2->power_prechg_eq_drivers.readOp.dynamic * 1e9 << endl;
            cout <<  "\tBitlines (nJ): " <<
                 fr->tag_array2->power_bitlines.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSense amplifier energy (nJ): " <<
                 fr->tag_array2->power_sense_amps.readOp.dynamic * 1e9 << endl;
            cout <<  "\tSub-array output driver (nJ): " <<
                 fr->tag_array2->power_output_drivers_at_subarray.readOp.dynamic * 1e9 << endl;
        }

        cout << endl << endl <<  "Area Components:" << endl << endl;
        /* Data array area stats */
        if (!(g_ip->pure_cam || g_ip->fully_assoc))
            cout <<  "  Data array: Area (mm2): " << fr->data_array2->area * 1e-6 << endl;
        else if (g_ip->pure_cam)
            cout <<  "  CAM array: Area (mm2): " << fr->data_array2->area * 1e-6 << endl;
        else
            cout <<  "  Fully associative cache array: Area (mm2): " << fr->data_array2->area * 1e-6 << endl;
        cout <<  "\tHeight (mm): " <<
             fr->data_array2->all_banks_height*1e-3 << endl;
        cout <<  "\tWidth (mm): " <<
             fr->data_array2->all_banks_width*1e-3 << endl;
        if (g_ip->print_detail) {
            cout <<  "\tArea efficiency (Memory cell area/Total area) - " <<
                 fr->data_array2->area_efficiency << " %" << endl;
            cout << "\t\tMAT Height (mm): " <<
                 fr->data_array2->mat_height*1e-3 << endl;
            cout << "\t\tMAT Length (mm): " <<
                 fr->data_array2->mat_length*1e-3 << endl;
            cout << "\t\tSubarray Height (mm): " <<
                 fr->data_array2->subarray_height*1e-3 << endl;
            cout << "\t\tSubarray Length (mm): " <<
                 fr->data_array2->subarray_length*1e-3 << endl;
        }

        /* Tag array area stats */
        if ((!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) &&
            !g_ip->is_main_mem) {
            cout << endl << "  Tag array: Area (mm2): " << fr->tag_array2->area * 1e-6 << endl;
            cout <<  "\tHeight (mm): " <<
                 fr->tag_array2->all_banks_height*1e-3 << endl;
            cout <<  "\tWidth (mm): " <<
                 fr->tag_array2->all_banks_width*1e-3 << endl;
            if (g_ip->print_detail) {
                cout <<  "\tArea efficiency (Memory cell area/Total area) - " <<
                     fr->tag_array2->area_efficiency << " %" << endl;
                cout << "\t\tMAT Height (mm): " <<
                     fr->tag_array2->mat_height*1e-3 << endl;
                cout << "\t\tMAT Length (mm): " <<
                     fr->tag_array2->mat_length*1e-3 << endl;
                cout << "\t\tSubarray Height (mm): " <<
                     fr->tag_array2->subarray_height*1e-3 << endl;
                cout << "\t\tSubarray Length (mm): " <<
                     fr->tag_array2->subarray_length*1e-3 << endl;
            }
        }
        Wire wpr;
        wpr.print_wire();
    }
}

//McPAT's plain interface, please keep !!!
uca_org_t cacti_interface(InputParameter * const local_interface) {
    uca_org_t fin_res;
    fin_res.valid = false;

    g_ip = local_interface;

    if (!g_ip->error_checking()) {
        exit(0);
    }

    init_tech_params(g_ip->F_sz_um, false);
    Wire winit; // Do not delete this line. It initializes wires.

    solve(&fin_res);

    return fin_res;
}

//McPAT's plain interface, please keep !!!
uca_org_t init_interface(InputParameter* const local_interface,
                         const string &name) {
    uca_org_t fin_res;
    fin_res.valid = false;

    g_ip = local_interface;

    if (!g_ip->error_checking(name)) {
        exit(0);
    }

    init_tech_params(g_ip->F_sz_um, false);
    Wire winit; // Do not delete this line. It initializes wires.
    return fin_res;
}

void reconfigure(InputParameter *local_interface, uca_org_t *fin_res)
{
  // Copy the InputParameter to global interface (g_ip) and do error checking.
  g_ip = local_interface;
  g_ip->error_checking();

  // Initialize technology parameters
  init_tech_params(g_ip->F_sz_um,false);

  Wire winit; // Do not delete this line. It initializes wires.

  // This corresponds to solve() in the initialization process.
  update(fin_res);
}
