#/bin/bash
trap 'kill -INT -$pid' INT

set -ex

resultFolder=MyoResults

mkdir -p $resultFolder

PS3='Please enter your choice: '

# In Myopic, the parameters should be set as follows:
#
#   - World size        = 3 ( which means 6 states )
#   - Initial state     = 6 ( uniform belief )
#   - Model Horizon     = 10
#   - Num. Experiments  = 1000
#   - POMCP iterations  = 1e4

wSize=3
iState=6
horiz=10
nExp=3000
iters=1e4

COLUMNS=1
# Should be 20 options (2 + 2 + 4 + 6 + 6)
options=(
    "Myopic POMCP-IR, MoB"
    "5-Hor  POMCP-IR, MoB"

    "Myopic RTBSS-IR, MoB"
    "3-Hor  RTBSS-IR, MoB"

    "Myopic RTBSSb,   MoB"
    "Myopic RTBSSb,   Entropy"
    "5-Hor  RTBSSb,   MoB"
    "5-Hor  RTBSSb,   Entropy"

    "Myopic rPOMCP,   MoB,     k=1"
    "Myopic rPOMCP,   MoB,     k=500"
    "Myopic rPOMCP,   MoB,     k=10000"
    "Myopic rPOMCP,   Entropy, k=1"
    "Myopic rPOMCP,   Entropy, k=500"
    "Myopic rPOMCP,   Entropy, k=10000"
    "5-Hor  rPOMCP,   MoB,     k=1"
    "5-Hor  rPOMCP,   MoB,     k=500"
    "5-Hor  rPOMCP,   MoB,     k=10000"
    "5-Hor  rPOMCP,   Entropy, k=1"
    "5-Hor  rPOMCP,   Entropy, k=500"
    "5-Hor  rPOMCP,   Entropy, k=10000"
)
select opt in "${options[@]}"
do
    case $opt in
        "Myopic POMCP-IR, MoB")
            name='myoMB'; solver=0; solverHorizon=1; k=0; break
            ;;
        "5-Hor  POMCP-IR, MoB")
            name='myoMB'; solver=0; solverHorizon=5; k=0; break
            ;;

#####################

        "Myopic RTBSS-IR, MoB")
            name='myoMB'; solver=2; solverHorizon=1; k=0; break
            ;;
        "3-Hor  RTBSS-IR, MoB")
            name='myoMB'; solver=2; solverHorizon=3; k=0; break
            ;;

#####################

        "Myopic RTBSSb,   MoB")
            name='myoMB'; solver=3; solverHorizon=1; k=0; break
            ;;
        "Myopic RTBSSb,   Entropy")
            name='myo';   solver=3; solverHorizon=1; k=0; break
            ;;
        "5-Hor  RTBSSb,   MoB")
            name='myoMB'; solver=3; solverHorizon=5; k=0; break
            ;;
        "5-Hor  RTBSSb,   Entropy")
            name='myo';   solver=3; solverHorizon=5; k=0; break
            ;;

#####################

        "Myopic rPOMCP,   MoB,     k=1")
            name='myoMB'; solver=1; solverHorizon=1; k=1; break
            ;;
        "Myopic rPOMCP,   MoB,     k=500")
            name='myoMB'; solver=1; solverHorizon=1; k=500; break
            ;;
        "Myopic rPOMCP,   MoB,     k=10000")
            name='myoMB'; solver=1; solverHorizon=1; k=10000; break
            ;;

        "Myopic rPOMCP,   Entropy, k=1")
            name='myo';   solver=1; solverHorizon=1; k=1; break
            ;;
        "Myopic rPOMCP,   Entropy, k=500")
            name='myo';   solver=1; solverHorizon=1; k=500; break
            ;;
        "Myopic rPOMCP,   Entropy, k=10000")
            name='myo';   solver=1; solverHorizon=1; k=10000; break
            ;;

        "5-Hor  rPOMCP,   MoB,     k=1")
            name='myoMB'; solver=1; solverHorizon=5; k=1; break
            ;;
        "5-Hor  rPOMCP,   MoB,     k=500")
            name='myoMB'; solver=1; solverHorizon=5; k=500; break
            ;;
        "5-Hor  rPOMCP,   MoB,     k=10000")
            name='myoMB'; solver=1; solverHorizon=5; k=10000; break
            ;;

        "5-Hor  rPOMCP,   Entropy, k=1")
            name='myo';   solver=1; solverHorizon=5; k=1; break
            ;;
        "5-Hor  rPOMCP,   Entropy, k=500")
            name='myo';   solver=1; solverHorizon=5; k=500; break
            ;;
        "5-Hor  rPOMCP,   Entropy, k=10000")
            name='myo';   solver=1; solverHorizon=5; k=10000; break
            ;;

        *) echo invalid option;;
    esac
done

# Give hard limit of 3 hours per command
timeout -s 2 10800 ./$name       $solver  $wSize   $iState     $solverHorizon     $horiz  $iters   $k     $nExp    $resultFolder/$name\_$solver\_$wSize\_$iState\_$solverHorizon\_$horiz\_$iters\_$k\_$nExp\.txt &
pid=$!
wait $pid

