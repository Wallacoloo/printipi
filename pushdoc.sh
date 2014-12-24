#!/bin/bash
#Whenever Travis CI runs on our repository
# we have the opportunity to have it update the documentation and push that to gh-pages
#used as reference: https://evansosenko.com/posts/automatic-publishing-github-pages-travis-ci/
#used as reference: http://awestruct.org/auto-deploy-to-github-pages/

set -e #exit script if any of the commands error
set -x #echo each command

deploy_branch="gh-pages"
repo=`git config remote.origin.url | sed "s/^git:/https:/"`
deploy_url=`echo $repo | sed "s|https://|https://$GH_TOKEN@|"`

#checkout the deploy branch in a temporary directory
pushd `mktemp -d`
git clone --branch $deploy_branch $repo .
git config user.name $GIT_NAME
git config user.email $GIT_EMAIL
#make changes
touch travis-test.txt
git add travis-test.txt
git commit -m"test Travis-CI push" || true #attempting to make a commit with no changes will raise an error
git push -q $deploy_url $deploy_branch
#cleanup
rm -rf $(pwd)
popd
