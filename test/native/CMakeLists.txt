#
# Host-tests. Elke suite is een losstaand C-programma dat zijn eigen
# PASS/FAIL-regels afdrukt en met een exitcode eindigt; ctest doet verder
# niets magisch.
#
# De tests zijn gedragstests, geen structuurtests: ze laten de solver
# tientallen gesimuleerde seconden rijden en kijken waar het voertuig
# eindigt, hoe ver het na aankomst nog zwerft en of het een muur raakt.
# Verschillende grenswaarden zijn gemeten voordat ze zijn vastgelegd.
#

set(RBT_TESTS
    vehicle_model    # stapresponsies, rate-limiet, draaistralen, wielposities
    model            # registry-integriteit: elk model beantwoordt elke aanroep
    path             # fillet-afronding, scherpe hoeken, ontaarde invoer
    mpc              # rooster-solver: volgen, bochten, stoeprand, eindpose
    mppi             # reproduceerbaarheid, gekleurde ruis, aankomst
    skid_steer       # slipfactor, gekoppelde wielsnelheidslimiet
    omni3            # holonoom model, wielpolytoop, heading-modi
    docking          # eindpose-precisie over de hele registry
)

foreach(t ${RBT_TESTS})
    add_executable(test_${t} test_${t}.c)
    target_link_libraries(test_${t} PRIVATE rbt_mpc)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
