#
# De planner: acht vertaaleenheden, geen afhankelijkheden buiten libm.
#
# Twee doelen uit dezelfde bronnen:
#   rbt_mpc         statisch, waar de tests tegenaan linken
#   rbt_mpc_shared  librbt_mpc.so, wat de Python-visualisatie inlaadt
#
# Dat ze dezelfde bronnen delen is het hele punt: wat je in de playground
# ziet bewegen, is de code die ook onder de tests ligt en straks op het
# voertuig draait.
#

set(RBT_MPC_SOURCES
    rbt_vehicle_model.c      # kinematisch fietsmodel met actuatordynamica
    rbt_model.c              # generieke modelinterface + registry
    rbt_model_ackermann.c
    rbt_model_skid_steer.c   # ook diff-drive (zelfde code, slipfactor 1)
    rbt_model_omni3.c
    rbt_mpc.c                # rooster-solver + gedeelde kostenfunctie
    rbt_mppi.c               # MPPI-solver
    rbt_path.c               # fillet-afronding en scherpe-hoek-splitsing
)

add_library(rbt_mpc STATIC ${RBT_MPC_SOURCES})
target_include_directories(rbt_mpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rbt_mpc PUBLIC m)

add_library(rbt_mpc_shared SHARED ${RBT_MPC_SOURCES})
target_include_directories(rbt_mpc_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rbt_mpc_shared PUBLIC m)
set_target_properties(rbt_mpc_shared PROPERTIES
    OUTPUT_NAME rbt_mpc
    POSITION_INDEPENDENT_CODE ON)
