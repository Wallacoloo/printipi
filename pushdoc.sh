#!/bin/bash
#Whenever Travis CI runs on our repository
# we have the opportunity to have it update the documentation and push that to gh-pages
#used as reference: http://awestruct.org/auto-deploy-to-github-pages/

set -e #exit script if any of the commands error

doc_branch="gh-pages"
repo=`git config remote.origin.url` | sed "s/^git:/https:/"

git remote set-url --push origin $repo
git remote set-branches --add origin $doc_branch
git fetch -q
git config user.name $GIT_NAME
git config user.email $GIT_EMAIL
git config credential.helper "store --file=.git/credentials"
echo "https://$GH_TOKEN:@github.com" >> .git/credentials
git branch $doc_branch origin/$doc_branch
touch travis-test.txt
git add travis-test.txt
git push origin $doc_branch
rm .git/credentials
