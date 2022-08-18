#Git documentation
##Configuring user environment
git config --global user.name "User Name"
git config --global user.email "Mail address"
git clone "Repository"

##Branches
git branch "Branch name" > Creates a new branch under that name
git checkout "Branch name" > Switches to that branch
git merge "Branch name" > Merges the specified branch with this branch
git branch -d "Branch name" > Deletes specified branch

##Commit commands
git commit > Creates a new commit
git commit -m "Details" > Records the snapshot permanently in version history
git revert "commit" > Undoes that particular commit by introducing a new commit
git reset --hard "Commit" > Moves to that commit and deletes everything beyond that
git add "File Name" > Adds a particular file to the commit

##Push
git push origin HEAD --force > Pushes the changes back to the remote repository
