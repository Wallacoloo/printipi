#!/bin/sh
#Run printipi for 5 minutes, gathering profile info. Any arguments will be relayed to printipi (eg the name of a file to print)
timeout 5m sudo perf record ./build/printipi $@

#create the profile dir, in case it doesn't already exist
mkdir -p profile
mv perf.data profile
pushd profile

#Now archive the profiling information:
perf archive

#must unzip the archive:
TMPDIR=`maketemp -d`
tar xvf perf.data.tar.bz2 -C $TMPDIR

#Now insert the perf.data into the archive
cp perf.data $TMPDIR

#and re-archive it
tar cjf $TMDIR -C perf.data.tar.bz2

#Done; return to original directory.
popd

echo "Profile info created in profile dir. Now switch to the profile branch and push your data"
