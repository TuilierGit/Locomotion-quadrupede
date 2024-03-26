# Locomotion-quadrupède

Welcome to this project.

To ease the development, I put everything else than `control.py` in the `.gitignore` so that we only need to push the `control.py` (all the other files will be local to your machine).
In other words, put all the other files in your repository but never push them (never add them to git).

I also suggest to leave the `main` branch like this and that everyone create a new branch for their own `control.py`.
This way, we can check what others are doing and also easily merge what's working while not interfering in each other's work.

Useful git commands :
-`git stash` : In case you edited the wrong branch and want to roll back to the last commit of the branch while saving your changes for later.
-`git stash apply` : Apply the changes you saved with `git stash` to your current branch. This is of course to be done after switching to the correct branch.
-`git checkout BRANCHNAME` : To switch to the branch named BRANCHNAME. Useful to check others' work or to go back to your own branch.
-`git checkout -b BRANCHNAME` : To create a new branch named BRANCHNAME and checkout to it immediately. This should be the first command to do after cloning the project.