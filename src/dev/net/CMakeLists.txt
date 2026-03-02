# src/dev/net/CMakeLists.txt

gem5_add_sources(
    # Ethernet infrastructure
    etherbus.cc
    etherswitch.cc
    etherdevice.cc
    etherdump.cc
    etherint.cc
    etherlink.cc
    etherpkt.cc
    ethertap.cc
    pktfifo.cc
    # Dist gem5
    dist_iface.cc
    dist_etherlink.cc
    tcp_iface.cc
    # Ethernet controllers
    i8254xGBe.cc
    ns_gige.cc
    sinic.cc
)

# --- Debug flags ---
gem5_add_debug_flag(Ethernet "Ethernet debugging")
gem5_add_debug_flag(EthernetCksum "EthernetCksum debugging")
gem5_add_debug_flag(EthernetDMA "EthernetDMA debugging")
gem5_add_debug_flag(EthernetData "EthernetData debugging")
gem5_add_debug_flag(EthernetDesc "EthernetDesc debugging")
gem5_add_debug_flag(EthernetEEPROM "EthernetEEPROM debugging")
gem5_add_debug_flag(EthernetIntr "EthernetIntr debugging")
gem5_add_debug_flag(EthernetPIO "EthernetPIO debugging")
gem5_add_debug_flag(EthernetSM "EthernetSM debugging")
gem5_add_debug_flag(DistEthernet "DistEthernet debugging")
gem5_add_debug_flag(DistEthernetPkt "DistEthernetPkt debugging")
gem5_add_debug_flag(DistEthernetCmd "DistEthernetCmd debugging")

# Compound flags
gem5_add_debug_flag(EthernetAll "All Ethernet debugging"
    COMPONENTS Ethernet EthernetPIO EthernetDMA EthernetData
               EthernetDesc EthernetIntr EthernetSM EthernetCksum
               EthernetEEPROM)
gem5_add_debug_flag(EthernetNoData "Ethernet debugging without data"
    COMPONENTS Ethernet EthernetPIO EthernetDesc EthernetIntr
               EthernetSM EthernetCksum)

# EtherTap requires linux/if_tun.h (not available on macOS).
# The C++ source (ethertap.cc) has #if HAVE_TUNTAP guards, but the
# SimObject registration must also be conditional to avoid generating
# param_EtherTap.cc on platforms without tun/tap support.
set(_ethernet_simobjects
    EtherLink DistEtherLink EtherBus EtherSwitch EtherTapBase
    EtherTapStub EtherDump EtherDevice IGbE EtherDevBase
    NSGigE Sinic)
if(CONF_HAVE_TUNTAP)
    list(APPEND _ethernet_simobjects EtherTap)
endif()
gem5_add_simobject(Ethernet.py SIM_OBJECTS ${_ethernet_simobjects})
