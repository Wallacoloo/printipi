#!/bin/bash
#Whenever Travis CI runs on our repository
# we have the opportunity to have it update the documentation and push that to gh-pages
#used as reference: https://evansosenko.com/posts/automatic-publishing-github-pages-travis-ci/
#used as reference: http://awestruct.org/auto-deploy-to-github-pages/

set -e #exit script if any of the commands error
set +x #DON'T echo the next few commands

deploy_branch="gh-pages"
repo=`git config remote.origin.url | sed "s/^git:/https:/"`
deploy_url=`echo $repo | sed "s|https://|https://$GH_TOKEN@|"`

gitroot=$PWD

#checkout the deploy branch in a temporary directory
pushd `mktemp -d`
git clone --branch $deploy_branch $repo .
git config user.name $GIT_NAME
git config user.email $GIT_EMAIL

set -x #echo future commands

#If running in Travis CI:
PATH=$PATH:/home/travis/.local/bin

#cldoc fix for https://github.com/jessevdk/cldoc/issues/2
#pushd /usr/bin/x86_64-linux-gnu
#sudo ln -s libclang.so.1 libclang.so
#popd

#build the documentation:
pushd $gitroot/src
make doc
popd
#copy the documentation into the gh-pages branch
git rm -rf **
cp -r $gitroot/doc/* .

#commit the changes
git add --all
git commit -m"Travis CI auto-update documentation" || true #attempting to make a commit with no changes will otherwise raise an error
git push -q $deploy_url $deploy_branch

#cleanup
rm -rf $(pwd)
popd
