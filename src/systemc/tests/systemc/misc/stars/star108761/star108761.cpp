/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  star108761.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE(test) {
  sc_in_clk clk;
  sc_in<bool> reset;
  sc_in<sc_uint<10> > inp;
  sc_out<sc_uint<10> > outp;

  SC_CTOR(test) {
    SC_CTHREAD(entry, clk.pos());
    reset_signal_is(reset,true);
    end_module();
  }

  void entry();
};


sc_uint<10> comp_mux(sc_uint<10> invar) {
  sc_uint<10> outvar;

  if(invar == 0) {
    outvar = 1023;
  } else if(invar == 1) {
    outvar = 1022;
  } else if(invar == 2) {
    outvar = 1021;
  } else if(invar == 3) {
    outvar = 1020;
  } else if(invar == 4) {
    outvar = 1019;
  } else if(invar == 5) {
    outvar = 1018;
  } else if(invar == 6) {
    outvar = 1017;
  } else if(invar == 7) {
    outvar = 1016;
  } else if(invar == 8) {
    outvar = 1015;
  } else if(invar == 9) {
    outvar = 1014;
  } else if(invar == 10) {
    outvar = 1013;
  } else if(invar == 11) {
    outvar = 1012;
  } else if(invar == 12) {
    outvar = 1011;
  } else if(invar == 13) {
    outvar = 1010;
  } else if(invar == 14) {
    outvar = 1009;
  } else if(invar == 15) {
    outvar = 1008;
  } else if(invar == 16) {
    outvar = 1007;
  } else if(invar == 17) {
    outvar = 1006;
  } else if(invar == 18) {
    outvar = 1005;
  } else if(invar == 19) {
    outvar = 1004;
  } else if(invar == 20) {
    outvar = 1003;
  } else if(invar == 21) {
    outvar = 1002;
  } else if(invar == 22) {
    outvar = 1001;
  } else if(invar == 23) {
    outvar = 1000;
  } else if(invar == 24) {
    outvar = 999;
  } else if(invar == 25) {
    outvar = 998;
  } else if(invar == 26) {
    outvar = 997;
  } else if(invar == 27) {
    outvar = 996;
  } else if(invar == 28) {
    outvar = 995;
  } else if(invar == 29) {
    outvar = 994;
  } else if(invar == 30) {
    outvar = 993;
  } else if(invar == 31) {
    outvar = 992;
  } else if(invar == 32) {
    outvar = 991;
  } else if(invar == 33) {
    outvar = 990;
  } else if(invar == 34) {
    outvar = 989;
  } else if(invar == 35) {
    outvar = 988;
  } else if(invar == 36) {
    outvar = 987;
  } else if(invar == 37) {
    outvar = 986;
  } else if(invar == 38) {
    outvar = 985;
  } else if(invar == 39) {
    outvar = 984;
  } else if(invar == 40) {
    outvar = 983;
  } else if(invar == 41) {
    outvar = 982;
  } else if(invar == 42) {
    outvar = 981;
  } else if(invar == 43) {
    outvar = 980;
  } else if(invar == 44) {
    outvar = 979;
  } else if(invar == 45) {
    outvar = 978;
  } else if(invar == 46) {
    outvar = 977;
  } else if(invar == 47) {
    outvar = 976;
  } else if(invar == 48) {
    outvar = 975;
  } else if(invar == 49) {
    outvar = 974;
  } else if(invar == 50) {
    outvar = 973;
  } else if(invar == 51) {
    outvar = 972;
  } else if(invar == 52) {
    outvar = 971;
  } else if(invar == 53) {
    outvar = 970;
  } else if(invar == 54) {
    outvar = 969;
  } else if(invar == 55) {
    outvar = 968;
  } else if(invar == 56) {
    outvar = 967;
  } else if(invar == 57) {
    outvar = 966;
  } else if(invar == 58) {
    outvar = 965;
  } else if(invar == 59) {
    outvar = 964;
  } else if(invar == 60) {
    outvar = 963;
  } else if(invar == 61) {
    outvar = 962;
  } else if(invar == 62) {
    outvar = 961;
  } else if(invar == 63) {
    outvar = 960;
  } else if(invar == 64) {
    outvar = 959;
  } else if(invar == 65) {
    outvar = 958;
  } else if(invar == 66) {
    outvar = 957;
  } else if(invar == 67) {
    outvar = 956;
  } else if(invar == 68) {
    outvar = 955;
  } else if(invar == 69) {
    outvar = 954;
  } else if(invar == 70) {
    outvar = 953;
  } else if(invar == 71) {
    outvar = 952;
  } else if(invar == 72) {
    outvar = 951;
  } else if(invar == 73) {
    outvar = 950;
  } else if(invar == 74) {
    outvar = 949;
  } else if(invar == 75) {
    outvar = 948;
  } else if(invar == 76) {
    outvar = 947;
  } else if(invar == 77) {
    outvar = 946;
  } else if(invar == 78) {
    outvar = 945;
  } else if(invar == 79) {
    outvar = 944;
  } else if(invar == 80) {
    outvar = 943;
  } else if(invar == 81) {
    outvar = 942;
  } else if(invar == 82) {
    outvar = 941;
  } else if(invar == 83) {
    outvar = 940;
  } else if(invar == 84) {
    outvar = 939;
  } else if(invar == 85) {
    outvar = 938;
  } else if(invar == 86) {
    outvar = 937;
  } else if(invar == 87) {
    outvar = 936;
  } else if(invar == 88) {
    outvar = 935;
  } else if(invar == 89) {
    outvar = 934;
  } else if(invar == 90) {
    outvar = 933;
  } else if(invar == 91) {
    outvar = 932;
  } else if(invar == 92) {
    outvar = 931;
  } else if(invar == 93) {
    outvar = 930;
  } else if(invar == 94) {
    outvar = 929;
  } else if(invar == 95) {
    outvar = 928;
  } else if(invar == 96) {
    outvar = 927;
  } else if(invar == 97) {
    outvar = 926;
  } else if(invar == 98) {
    outvar = 925;
  } else if(invar == 99) {
    outvar = 924;
  } else if(invar == 100) {
    outvar = 923;
  } else if(invar == 101) {
    outvar = 922;
  } else if(invar == 102) {
    outvar = 921;
  } else if(invar == 103) {
    outvar = 920;
  } else if(invar == 104) {
    outvar = 919;
  } else if(invar == 105) {
    outvar = 918;
  } else if(invar == 106) {
    outvar = 917;
  } else if(invar == 107) {
    outvar = 916;
  } else if(invar == 108) {
    outvar = 915;
  } else if(invar == 109) {
    outvar = 914;
  } else if(invar == 110) {
    outvar = 913;
  } else if(invar == 111) {
    outvar = 912;
  } else if(invar == 112) {
    outvar = 911;
  } else if(invar == 113) {
    outvar = 910;
  } else if(invar == 114) {
    outvar = 909;
  } else if(invar == 115) {
    outvar = 908;
  } else if(invar == 116) {
    outvar = 907;
  } else if(invar == 117) {
    outvar = 906;
  } else if(invar == 118) {
    outvar = 905;
  } else if(invar == 119) {
    outvar = 904;
  } else if(invar == 120) {
    outvar = 903;
  } else if(invar == 121) {
    outvar = 902;
#if !defined( _MSC_VER )
  // 'cl' compiler limit: blocks nested too deeply
  } else if(invar == 122) {
    outvar = 901;
  } else if(invar == 123) {
    outvar = 900;
  } else if(invar == 124) {
    outvar = 899;
  } else if(invar == 125) {
    outvar = 898;
  } else if(invar == 126) {
    outvar = 897;
  } else if(invar == 127) {
    outvar = 896;
  } else if(invar == 128) {
    outvar = 895;
  } else if(invar == 129) {
    outvar = 894;
  } else if(invar == 130) {
    outvar = 893;
  } else if(invar == 131) {
    outvar = 892;
  } else if(invar == 132) {
    outvar = 891;
  } else if(invar == 133) {
    outvar = 890;
  } else if(invar == 134) {
    outvar = 889;
  } else if(invar == 135) {
    outvar = 888;
  } else if(invar == 136) {
    outvar = 887;
  } else if(invar == 137) {
    outvar = 886;
  } else if(invar == 138) {
    outvar = 885;
  } else if(invar == 139) {
    outvar = 884;
  } else if(invar == 140) {
    outvar = 883;
  } else if(invar == 141) {
    outvar = 882;
  } else if(invar == 142) {
    outvar = 881;
  } else if(invar == 143) {
    outvar = 880;
  } else if(invar == 144) {
    outvar = 879;
  } else if(invar == 145) {
    outvar = 878;
  } else if(invar == 146) {
    outvar = 877;
  } else if(invar == 147) {
    outvar = 876;
  } else if(invar == 148) {
    outvar = 875;
  } else if(invar == 149) {
    outvar = 874;
  } else if(invar == 150) {
    outvar = 873;
  } else if(invar == 151) {
    outvar = 872;
  } else if(invar == 152) {
    outvar = 871;
  } else if(invar == 153) {
    outvar = 870;
  } else if(invar == 154) {
    outvar = 869;
  } else if(invar == 155) {
    outvar = 868;
  } else if(invar == 156) {
    outvar = 867;
  } else if(invar == 157) {
    outvar = 866;
  } else if(invar == 158) {
    outvar = 865;
  } else if(invar == 159) {
    outvar = 864;
  } else if(invar == 160) {
    outvar = 863;
  } else if(invar == 161) {
    outvar = 862;
  } else if(invar == 162) {
    outvar = 861;
  } else if(invar == 163) {
    outvar = 860;
  } else if(invar == 164) {
    outvar = 859;
  } else if(invar == 165) {
    outvar = 858;
  } else if(invar == 166) {
    outvar = 857;
  } else if(invar == 167) {
    outvar = 856;
  } else if(invar == 168) {
    outvar = 855;
  } else if(invar == 169) {
    outvar = 854;
  } else if(invar == 170) {
    outvar = 853;
  } else if(invar == 171) {
    outvar = 852;
  } else if(invar == 172) {
    outvar = 851;
  } else if(invar == 173) {
    outvar = 850;
  } else if(invar == 174) {
    outvar = 849;
  } else if(invar == 175) {
    outvar = 848;
  } else if(invar == 176) {
    outvar = 847;
  } else if(invar == 177) {
    outvar = 846;
  } else if(invar == 178) {
    outvar = 845;
  } else if(invar == 179) {
    outvar = 844;
  } else if(invar == 180) {
    outvar = 843;
  } else if(invar == 181) {
    outvar = 842;
  } else if(invar == 182) {
    outvar = 841;
  } else if(invar == 183) {
    outvar = 840;
  } else if(invar == 184) {
    outvar = 839;
  } else if(invar == 185) {
    outvar = 838;
  } else if(invar == 186) {
    outvar = 837;
  } else if(invar == 187) {
    outvar = 836;
  } else if(invar == 188) {
    outvar = 835;
  } else if(invar == 189) {
    outvar = 834;
  } else if(invar == 190) {
    outvar = 833;
  } else if(invar == 191) {
    outvar = 832;
  } else if(invar == 192) {
    outvar = 831;
  } else if(invar == 193) {
    outvar = 830;
  } else if(invar == 194) {
    outvar = 829;
  } else if(invar == 195) {
    outvar = 828;
  } else if(invar == 196) {
    outvar = 827;
  } else if(invar == 197) {
    outvar = 826;
  } else if(invar == 198) {
    outvar = 825;
  } else if(invar == 199) {
    outvar = 824;
  } else if(invar == 200) {
    outvar = 823;
  } else if(invar == 201) {
    outvar = 822;
  } else if(invar == 202) {
    outvar = 821;
  } else if(invar == 203) {
    outvar = 820;
  } else if(invar == 204) {
    outvar = 819;
  } else if(invar == 205) {
    outvar = 818;
  } else if(invar == 206) {
    outvar = 817;
  } else if(invar == 207) {
    outvar = 816;
  } else if(invar == 208) {
    outvar = 815;
  } else if(invar == 209) {
    outvar = 814;
  } else if(invar == 210) {
    outvar = 813;
  } else if(invar == 211) {
    outvar = 812;
  } else if(invar == 212) {
    outvar = 811;
  } else if(invar == 213) {
    outvar = 810;
  } else if(invar == 214) {
    outvar = 809;
  } else if(invar == 215) {
    outvar = 808;
  } else if(invar == 216) {
    outvar = 807;
  } else if(invar == 217) {
    outvar = 806;
  } else if(invar == 218) {
    outvar = 805;
  } else if(invar == 219) {
    outvar = 804;
  } else if(invar == 220) {
    outvar = 803;
  } else if(invar == 221) {
    outvar = 802;
  } else if(invar == 222) {
    outvar = 801;
  } else if(invar == 223) {
    outvar = 800;
  } else if(invar == 224) {
    outvar = 799;
  } else if(invar == 225) {
    outvar = 798;
  } else if(invar == 226) {
    outvar = 797;
  } else if(invar == 227) {
    outvar = 796;
  } else if(invar == 228) {
    outvar = 795;
  } else if(invar == 229) {
    outvar = 794;
  } else if(invar == 230) {
    outvar = 793;
  } else if(invar == 231) {
    outvar = 792;
  } else if(invar == 232) {
    outvar = 791;
  } else if(invar == 233) {
    outvar = 790;
  } else if(invar == 234) {
    outvar = 789;
  } else if(invar == 235) {
    outvar = 788;
  } else if(invar == 236) {
    outvar = 787;
  } else if(invar == 237) {
    outvar = 786;
  } else if(invar == 238) {
    outvar = 785;
  } else if(invar == 239) {
    outvar = 784;
  } else if(invar == 240) {
    outvar = 783;
  } else if(invar == 241) {
    outvar = 782;
  } else if(invar == 242) {
    outvar = 781;
  } else if(invar == 243) {
    outvar = 780;
  } else if(invar == 244) {
    outvar = 779;
  } else if(invar == 245) {
    outvar = 778;
  } else if(invar == 246) {
    outvar = 777;
  } else if(invar == 247) {
    outvar = 776;
  } else if(invar == 248) {
    outvar = 775;
  } else if(invar == 249) {
    outvar = 774;
  } else if(invar == 250) {
    outvar = 773;
  } else if(invar == 251) {
    outvar = 772;
  } else if(invar == 252) {
    outvar = 771;
  } else if(invar == 253) {
    outvar = 770;
  } else if(invar == 254) {
    outvar = 769;
  } else if(invar == 255) {
    outvar = 768;
  } else if(invar == 256) {
    outvar = 767;
  } else if(invar == 257) {
    outvar = 766;
  } else if(invar == 258) {
    outvar = 765;
  } else if(invar == 259) {
    outvar = 764;
  } else if(invar == 260) {
    outvar = 763;
  } else if(invar == 261) {
    outvar = 762;
  } else if(invar == 262) {
    outvar = 761;
  } else if(invar == 263) {
    outvar = 760;
  } else if(invar == 264) {
    outvar = 759;
  } else if(invar == 265) {
    outvar = 758;
  } else if(invar == 266) {
    outvar = 757;
  } else if(invar == 267) {
    outvar = 756;
  } else if(invar == 268) {
    outvar = 755;
  } else if(invar == 269) {
    outvar = 754;
  } else if(invar == 270) {
    outvar = 753;
  } else if(invar == 271) {
    outvar = 752;
  } else if(invar == 272) {
    outvar = 751;
  } else if(invar == 273) {
    outvar = 750;
  } else if(invar == 274) {
    outvar = 749;
  } else if(invar == 275) {
    outvar = 748;
  } else if(invar == 276) {
    outvar = 747;
  } else if(invar == 277) {
    outvar = 746;
  } else if(invar == 278) {
    outvar = 745;
  } else if(invar == 279) {
    outvar = 744;
  } else if(invar == 280) {
    outvar = 743;
  } else if(invar == 281) {
    outvar = 742;
  } else if(invar == 282) {
    outvar = 741;
  } else if(invar == 283) {
    outvar = 740;
  } else if(invar == 284) {
    outvar = 739;
  } else if(invar == 285) {
    outvar = 738;
  } else if(invar == 286) {
    outvar = 737;
  } else if(invar == 287) {
    outvar = 736;
  } else if(invar == 288) {
    outvar = 735;
  } else if(invar == 289) {
    outvar = 734;
  } else if(invar == 290) {
    outvar = 733;
  } else if(invar == 291) {
    outvar = 732;
  } else if(invar == 292) {
    outvar = 731;
  } else if(invar == 293) {
    outvar = 730;
  } else if(invar == 294) {
    outvar = 729;
  } else if(invar == 295) {
    outvar = 728;
  } else if(invar == 296) {
    outvar = 727;
  } else if(invar == 297) {
    outvar = 726;
  } else if(invar == 298) {
    outvar = 725;
  } else if(invar == 299) {
    outvar = 724;
  } else if(invar == 300) {
    outvar = 723;
  } else if(invar == 301) {
    outvar = 722;
  } else if(invar == 302) {
    outvar = 721;
  } else if(invar == 303) {
    outvar = 720;
  } else if(invar == 304) {
    outvar = 719;
  } else if(invar == 305) {
    outvar = 718;
  } else if(invar == 306) {
    outvar = 717;
  } else if(invar == 307) {
    outvar = 716;
  } else if(invar == 308) {
    outvar = 715;
  } else if(invar == 309) {
    outvar = 714;
  } else if(invar == 310) {
    outvar = 713;
  } else if(invar == 311) {
    outvar = 712;
  } else if(invar == 312) {
    outvar = 711;
  } else if(invar == 313) {
    outvar = 710;
  } else if(invar == 314) {
    outvar = 709;
  } else if(invar == 315) {
    outvar = 708;
  } else if(invar == 316) {
    outvar = 707;
  } else if(invar == 317) {
    outvar = 706;
  } else if(invar == 318) {
    outvar = 705;
  } else if(invar == 319) {
    outvar = 704;
  } else if(invar == 320) {
    outvar = 703;
  } else if(invar == 321) {
    outvar = 702;
  } else if(invar == 322) {
    outvar = 701;
  } else if(invar == 323) {
    outvar = 700;
  } else if(invar == 324) {
    outvar = 699;
  } else if(invar == 325) {
    outvar = 698;
  } else if(invar == 326) {
    outvar = 697;
  } else if(invar == 327) {
    outvar = 696;
  } else if(invar == 328) {
    outvar = 695;
  } else if(invar == 329) {
    outvar = 694;
  } else if(invar == 330) {
    outvar = 693;
  } else if(invar == 331) {
    outvar = 692;
  } else if(invar == 332) {
    outvar = 691;
  } else if(invar == 333) {
    outvar = 690;
  } else if(invar == 334) {
    outvar = 689;
  } else if(invar == 335) {
    outvar = 688;
  } else if(invar == 336) {
    outvar = 687;
  } else if(invar == 337) {
    outvar = 686;
  } else if(invar == 338) {
    outvar = 685;
  } else if(invar == 339) {
    outvar = 684;
  } else if(invar == 340) {
    outvar = 683;
  } else if(invar == 341) {
    outvar = 682;
  } else if(invar == 342) {
    outvar = 681;
  } else if(invar == 343) {
    outvar = 680;
  } else if(invar == 344) {
    outvar = 679;
  } else if(invar == 345) {
    outvar = 678;
  } else if(invar == 346) {
    outvar = 677;
  } else if(invar == 347) {
    outvar = 676;
  } else if(invar == 348) {
    outvar = 675;
  } else if(invar == 349) {
    outvar = 674;
  } else if(invar == 350) {
    outvar = 673;
  } else if(invar == 351) {
    outvar = 672;
  } else if(invar == 352) {
    outvar = 671;
  } else if(invar == 353) {
    outvar = 670;
  } else if(invar == 354) {
    outvar = 669;
  } else if(invar == 355) {
    outvar = 668;
  } else if(invar == 356) {
    outvar = 667;
  } else if(invar == 357) {
    outvar = 666;
  } else if(invar == 358) {
    outvar = 665;
  } else if(invar == 359) {
    outvar = 664;
  } else if(invar == 360) {
    outvar = 663;
  } else if(invar == 361) {
    outvar = 662;
  } else if(invar == 362) {
    outvar = 661;
  } else if(invar == 363) {
    outvar = 660;
  } else if(invar == 364) {
    outvar = 659;
  } else if(invar == 365) {
    outvar = 658;
  } else if(invar == 366) {
    outvar = 657;
  } else if(invar == 367) {
    outvar = 656;
  } else if(invar == 368) {
    outvar = 655;
  } else if(invar == 369) {
    outvar = 654;
  } else if(invar == 370) {
    outvar = 653;
  } else if(invar == 371) {
    outvar = 652;
  } else if(invar == 372) {
    outvar = 651;
  } else if(invar == 373) {
    outvar = 650;
  } else if(invar == 374) {
    outvar = 649;
  } else if(invar == 375) {
    outvar = 648;
  } else if(invar == 376) {
    outvar = 647;
  } else if(invar == 377) {
    outvar = 646;
  } else if(invar == 378) {
    outvar = 645;
  } else if(invar == 379) {
    outvar = 644;
  } else if(invar == 380) {
    outvar = 643;
  } else if(invar == 381) {
    outvar = 642;
  } else if(invar == 382) {
    outvar = 641;
  } else if(invar == 383) {
    outvar = 640;
  } else if(invar == 384) {
    outvar = 639;
  } else if(invar == 385) {
    outvar = 638;
  } else if(invar == 386) {
    outvar = 637;
  } else if(invar == 387) {
    outvar = 636;
  } else if(invar == 388) {
    outvar = 635;
  } else if(invar == 389) {
    outvar = 634;
  } else if(invar == 390) {
    outvar = 633;
  } else if(invar == 391) {
    outvar = 632;
  } else if(invar == 392) {
    outvar = 631;
  } else if(invar == 393) {
    outvar = 630;
  } else if(invar == 394) {
    outvar = 629;
  } else if(invar == 395) {
    outvar = 628;
  } else if(invar == 396) {
    outvar = 627;
  } else if(invar == 397) {
    outvar = 626;
  } else if(invar == 398) {
    outvar = 625;
  } else if(invar == 399) {
    outvar = 624;
  } else if(invar == 400) {
    outvar = 623;
  } else if(invar == 401) {
    outvar = 622;
  } else if(invar == 402) {
    outvar = 621;
  } else if(invar == 403) {
    outvar = 620;
  } else if(invar == 404) {
    outvar = 619;
  } else if(invar == 405) {
    outvar = 618;
  } else if(invar == 406) {
    outvar = 617;
  } else if(invar == 407) {
    outvar = 616;
  } else if(invar == 408) {
    outvar = 615;
  } else if(invar == 409) {
    outvar = 614;
  } else if(invar == 410) {
    outvar = 613;
  } else if(invar == 411) {
    outvar = 612;
  } else if(invar == 412) {
    outvar = 611;
  } else if(invar == 413) {
    outvar = 610;
  } else if(invar == 414) {
    outvar = 609;
  } else if(invar == 415) {
    outvar = 608;
  } else if(invar == 416) {
    outvar = 607;
  } else if(invar == 417) {
    outvar = 606;
  } else if(invar == 418) {
    outvar = 605;
  } else if(invar == 419) {
    outvar = 604;
  } else if(invar == 420) {
    outvar = 603;
  } else if(invar == 421) {
    outvar = 602;
  } else if(invar == 422) {
    outvar = 601;
  } else if(invar == 423) {
    outvar = 600;
  } else if(invar == 424) {
    outvar = 599;
  } else if(invar == 425) {
    outvar = 598;
  } else if(invar == 426) {
    outvar = 597;
  } else if(invar == 427) {
    outvar = 596;
  } else if(invar == 428) {
    outvar = 595;
  } else if(invar == 429) {
    outvar = 594;
  } else if(invar == 430) {
    outvar = 593;
  } else if(invar == 431) {
    outvar = 592;
  } else if(invar == 432) {
    outvar = 591;
  } else if(invar == 433) {
    outvar = 590;
  } else if(invar == 434) {
    outvar = 589;
  } else if(invar == 435) {
    outvar = 588;
  } else if(invar == 436) {
    outvar = 587;
  } else if(invar == 437) {
    outvar = 586;
  } else if(invar == 438) {
    outvar = 585;
  } else if(invar == 439) {
    outvar = 584;
  } else if(invar == 440) {
    outvar = 583;
  } else if(invar == 441) {
    outvar = 582;
  } else if(invar == 442) {
    outvar = 581;
  } else if(invar == 443) {
    outvar = 580;
  } else if(invar == 444) {
    outvar = 579;
  } else if(invar == 445) {
    outvar = 578;
  } else if(invar == 446) {
    outvar = 577;
  } else if(invar == 447) {
    outvar = 576;
  } else if(invar == 448) {
    outvar = 575;
  } else if(invar == 449) {
    outvar = 574;
  } else if(invar == 450) {
    outvar = 573;
  } else if(invar == 451) {
    outvar = 572;
  } else if(invar == 452) {
    outvar = 571;
  } else if(invar == 453) {
    outvar = 570;
  } else if(invar == 454) {
    outvar = 569;
  } else if(invar == 455) {
    outvar = 568;
  } else if(invar == 456) {
    outvar = 567;
  } else if(invar == 457) {
    outvar = 566;
  } else if(invar == 458) {
    outvar = 565;
  } else if(invar == 459) {
    outvar = 564;
  } else if(invar == 460) {
    outvar = 563;
  } else if(invar == 461) {
    outvar = 562;
  } else if(invar == 462) {
    outvar = 561;
  } else if(invar == 463) {
    outvar = 560;
  } else if(invar == 464) {
    outvar = 559;
  } else if(invar == 465) {
    outvar = 558;
  } else if(invar == 466) {
    outvar = 557;
  } else if(invar == 467) {
    outvar = 556;
  } else if(invar == 468) {
    outvar = 555;
  } else if(invar == 469) {
    outvar = 554;
  } else if(invar == 470) {
    outvar = 553;
  } else if(invar == 471) {
    outvar = 552;
  } else if(invar == 472) {
    outvar = 551;
  } else if(invar == 473) {
    outvar = 550;
  } else if(invar == 474) {
    outvar = 549;
  } else if(invar == 475) {
    outvar = 548;
  } else if(invar == 476) {
    outvar = 547;
  } else if(invar == 477) {
    outvar = 546;
  } else if(invar == 478) {
    outvar = 545;
  } else if(invar == 479) {
    outvar = 544;
  } else if(invar == 480) {
    outvar = 543;
  } else if(invar == 481) {
    outvar = 542;
  } else if(invar == 482) {
    outvar = 541;
  } else if(invar == 483) {
    outvar = 540;
  } else if(invar == 484) {
    outvar = 539;
  } else if(invar == 485) {
    outvar = 538;
  } else if(invar == 486) {
    outvar = 537;
  } else if(invar == 487) {
    outvar = 536;
  } else if(invar == 488) {
    outvar = 535;
  } else if(invar == 489) {
    outvar = 534;
  } else if(invar == 490) {
    outvar = 533;
  } else if(invar == 491) {
    outvar = 532;
  } else if(invar == 492) {
    outvar = 531;
  } else if(invar == 493) {
    outvar = 530;
  } else if(invar == 494) {
    outvar = 529;
  } else if(invar == 495) {
    outvar = 528;
  } else if(invar == 496) {
    outvar = 527;
  } else if(invar == 497) {
    outvar = 526;
  } else if(invar == 498) {
    outvar = 525;
  } else if(invar == 499) {
    outvar = 524;
  } else if(invar == 500) {
    outvar = 523;
  } else if(invar == 501) {
    outvar = 522;
  } else if(invar == 502) {
    outvar = 521;
  } else if(invar == 503) {
    outvar = 520;
  } else if(invar == 504) {
    outvar = 519;
  } else if(invar == 505) {
    outvar = 518;
  } else if(invar == 506) {
    outvar = 517;
  } else if(invar == 507) {
    outvar = 516;
  } else if(invar == 508) {
    outvar = 515;
  } else if(invar == 509) {
    outvar = 514;
  } else if(invar == 510) {
    outvar = 513;
  } else if(invar == 511) {
    outvar = 512;
  } else if(invar == 512) {
    outvar = 511;
#endif
  }  return outvar;
}

void test::entry() {
  sc_uint<10> tmp;

  outp.write(0);
  wait();
  while(1) {
    // tmp = comp_mux(inp);
    tmp = comp_mux(inp.read());
    outp.write(tmp);
    wait();
  }
}
