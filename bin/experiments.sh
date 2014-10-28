#/bin/bash

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
nExp=1000
iters=1e4

COLUMNS=1
options=(
    "Myopic POMCP-IR, MoB"
    "5-Hor  POMCP-IR, MoB"

    "Myopic RTBSS-IR, MoB"
    "5-Hor  RTBSS-IR, MoB"

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
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     0  $wSize   $iState     1     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_0_$wSize\_$iState\_1_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "5-Hor  POMCP-IR, MoB")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     0  $wSize   $iState     5     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_0_$wSize\_$iState\_5_$horiz\_$iters\_$nExp\.txt
            break
            ;;

#####################

        "Myopic RTBSS-IR, MoB")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     2  $wSize   $iState     1     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_2_$wSize\_$iState\_1_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "5-Hor  RTBSS-IR, MoB")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     2  $wSize   $iState     5     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_2_$wSize\_$iState\_5_$horiz\_$iters\_$nExp\.txt
            break
            ;;

#####################

        "Myopic RTBSSb,   MoB")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     3  $wSize   $iState     1     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_3_$wSize\_$iState\_1_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "Myopic RTBSSb,   Entropy")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myo       3  $wSize   $iState     1     $horiz  $iters   0     $nExp    $resultFolder/myo_EE_3_$wSize\_$iState\_1_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "5-Hor  RTBSSb,   MoB")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     3  $wSize   $iState     5     $horiz  $iters   0     $nExp    $resultFolder/myo_MB_3_$wSize\_$iState\_5_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "5-Hor  RTBSSb,   Entropy")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myo       3  $wSize   $iState     5     $horiz  $iters   0     $nExp    $resultFolder/myo_EE_3_$wSize\_$iState\_5_$horiz\_$iters\_$nExp\.txt
            break
            ;;

#####################

        "Myopic rPOMCP,   MoB,     k=1")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     1  $wSize   $iState     1     $horiz  $iters   1    $nExp    $resultFolder/myo_1_$wSize\_$iState\_1_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "Myopic rPOMCP,   MoB,     k=500")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     1  $wSize   $iState     1     $horiz  $iters  500    $nExp    $resultFolder/myo_1_$wSize\_$iState\_500_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        "Myopic rPOMCP,   MoB,     k=10000")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myoMB     1  $wSize   $iState     1     $horiz  $iters 10000    $nExp    $resultFolder/myo_1_$wSize\_$iState\_10000_$horiz\_$iters\_$nExp\.txt
            break
            ;;

### TODO: CONTINUE FROM HERE:



        "5-Hor  rPOMCP,   Entropy")
            # ./myo  solv gridSize initState solvHor modelHor  iters   k    numExp          filename
            ./myo       1  $wSize   $iState     5     $horiz  $iters   1     $nExp    $resultFolder/myo_1_$wSize\_$iState\_5_$horiz\_$iters\_$nExp\.txt
            break
            ;;
        *) echo invalid option;;
    esac
done

