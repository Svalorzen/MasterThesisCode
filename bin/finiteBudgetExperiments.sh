#/bin/bash
trap 'kill -INT -$pid' INT

set -ex

resultFolder=FiniteBudgetResults

mkdir -p $resultFolder

PS3='Please enter your choice: '

# In FiniteBudget, the parameters should be set as follows:
#
#   - World size        = 4 
#   - Initial state     = 0 
#   - Model Horizon     = 15
#   - Num. Experiments  = 3000
#   - POMCP iterations  = 1e4
#   - Budget            = 4
#   - leftP             = 0.5/0.8

wSize=4
iState=0
horiz=15
nExp=3000
iters=1e4
budget=4
pLeft=0.5

COLUMNS=1
# Should be 16 options (2 + 2 + 4 + 2 + 6)
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
    "Myopic rPOMCP,   Entropy, k=1"
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
            name='fbMB'; solver=0; solverHorizon=1; k=0; break
            ;;
        "5-Hor  POMCP-IR, MoB")
            name='fbMB'; solver=0; solverHorizon=5; k=0; break
            ;;

#####################

        "Myopic RTBSS-IR, MoB")
            name='fbMB'; solver=2; solverHorizon=1; k=0; break
            ;;
        "3-Hor  RTBSS-IR, MoB")
            name='fbMB'; solver=2; solverHorizon=3; k=0; break
            ;;

#####################

        "Myopic RTBSSb,   MoB")
            name='fbMB'; solver=3; solverHorizon=1; k=0; break
            ;;
        "Myopic RTBSSb,   Entropy")
            name='fb';   solver=3; solverHorizon=1; k=0; break
            ;;
        "5-Hor  RTBSSb,   MoB")
            name='fbMB'; solver=3; solverHorizon=5; k=0; break
            ;;
        "5-Hor  RTBSSb,   Entropy")
            name='fb';   solver=3; solverHorizon=5; k=0; break
            ;;

#####################

        "Myopic rPOMCP,   MoB,     k=1")
            name='fbMB'; solver=1; solverHorizon=1; k=1; break
            ;;

        "Myopic rPOMCP,   Entropy, k=1")
            name='fb';   solver=1; solverHorizon=1; k=1; break
            ;;

        "5-Hor  rPOMCP,   MoB,     k=1")
            name='fbMB'; solver=1; solverHorizon=5; k=1; break
            ;;
        "5-Hor  rPOMCP,   MoB,     k=500")
            name='fbMB'; solver=1; solverHorizon=5; k=500; break
            ;;
        "5-Hor  rPOMCP,   MoB,     k=10000")
            name='fbMB'; solver=1; solverHorizon=5; k=10000; break
            ;;

        "5-Hor  rPOMCP,   Entropy, k=1")
            name='fb';   solver=1; solverHorizon=5; k=1; break
            ;;
        "5-Hor  rPOMCP,   Entropy, k=500")
            name='fb';   solver=1; solverHorizon=5; k=500; break
            ;;
        "5-Hor  rPOMCP,   Entropy, k=10000")
            name='fb';   solver=1; solverHorizon=5; k=10000; break
            ;;

        *) echo invalid option;;
    esac
done

# Give hard limit of 3 hours per command
timeout -s 2 10800 ./$name       $solver  $wSize   $iState     $solverHorizon     $horiz  $iters   $k     $nExp    $resultFolder/$name\_$solver\_$wSize\_$iState\_$solverHorizon\_$horiz\_$iters\_$k\_$nExp\_$budget\_$pLeft\.txt $budget $pLeft & 
pid=$!
wait $pid

