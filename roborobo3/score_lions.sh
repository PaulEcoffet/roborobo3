
#shopt -s globstar

# Variable definition

export SDL_VIDEODRIVER=dummy
path=""
ncpu=0
python=/Users/paulecoffet/opt/anaconda3/bin/python

for p in /Users/paulecoffet/Documents/isir/These/data/logs/lionscross10-procbtest/lions-tol0.7-pop100-fake_true-fakecoef_1-b10-/lion_megabig/*/*/*/*/rep00/
do
   echo $p
   $python playfromgenome.py -g 1499 -p $p -b +logScore true +logEveryXGen 1 +gDisplayMode 2&
   let "ncpu=ncpu+1"
   if [[ $ncpu == 5 ]]
   then
       wait
       ncpu=0
   fi
done
wait
echo "Over"
