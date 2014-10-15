#!/bin/sh
#Run printipi for 5 minutes, gathering profile info. Any arguments will be relayed to printipi (eg the name of a file to print)
#-g flag tells perf to record callgraph information
sudo timeout --signal=SIGINT 5m perf record -g ./build/printipi $@

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
sudo cp perf.data $TMPDIR
#make ourselves the owner of the archived perf.data
#Note: the original perf.data must be owned by the user who created it, otherwise `perf archive` complains
sudo chown "$USER" $TMPDIR/perf.data

#backup old archive
sudo mv perf.data.tar.bz2 perf.data.tar.bz2.back
#and re-archive it
tar cjf perf.data.tar.bz2 -C $TMPDIR .

#Done; return to original directory & cleanup
sudo rm perf.data.tar.bz2.back
rm -rf $TMPDIR
popd

echo "Profile info created in profile dir. Now switch to the profile branch and push your data."
echo "e.g. 'cd profile && git checkout profile && git add perf.data.tar.bz2 && git push origin profile && git checkout master'"
echo "To run the profile report on any machine, type 'tar xvf perf.data.tar.bz2 -C ~/.debug' followed by 'perf report -i ~/.debug/perf.data'"
