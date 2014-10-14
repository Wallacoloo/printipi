#!/bin/sh
#Run printipi for 5 minutes, gathering profile info. Any arguments will be relayed to printipi (eg the name of a file to print)
sudo timeout --signal=SIGINT 5m perf record ./build/printipi $@

#Give ourselves ownership of perf.data, rather than root (from sudo)
chown "$USER" perf.data

#create the profile dir, in case it doesn't already exist
mkdir -p profile
mv perf.data profile
pushd profile

#archive the profiling information:
#must be superuser, because data was inserted into ~/.debug by the perf command running as superuser.
sudo perf archive

#unzip the archive:
TMPDIR=`mktemp -d`
tar xvf perf.data.tar.bz2 -C $TMPDIR

#insert perf.data into the archive
cp perf.data $TMPDIR

#backup old archive
sudo mv perf.data.tar.bz2 perf.data.tar.bz2.back
#and re-archive it
tar cjf perf.data.tar.bz2 -C $TMDIR .

#Done; return to original directory.
popd

echo "Profile info created in profile dir. Now switch to the profile branch and push your data"
