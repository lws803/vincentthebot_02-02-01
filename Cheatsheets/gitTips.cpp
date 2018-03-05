// --------------------- For simple operations ---------------------

$ git init  // This initialises git in current folder 

$ git status // Shows you what file is currently tracked or untracked 

$ git add <FILENAME> // Stages the file for commit 

$ git commit -m "YOUR MESSAGE" // Commits the file change locally 

$ git diff // show changes that were made to what file

$ git log // See a log of all commits made with the commit hashcode 


// --------------------- For more advanced operations ---------------

$ git checkout -- <FILENAMES> // reset THOSE files back to head of master 

$ git checkout <COMMIT HASH> -- <FILENAMES> // reset THOSE files back to whatever point in time (commit hash)

// --------------------- Setting up remote repo  --------------------

$ git remote add origin <URL of git repo> // push the commits in local repo to the remote repo 

$ git push -u origin master // Push local commits to remote repo 

$ git fetch // To fetch changes 

$ git merge // To merge changes in remote repo with local repo 

/* Note that git fetch + merge is the equivalent of git pull */

$ git pull // To pull changes from remote repo to local repo 


// --------------------- Resolve conflicts   ------------------------

$ git commit 
$ git push 
// The above two steps are crucial so that git can recognise what the conflicts are
// You will see an error message here telling you of mismatch 

$ git pull // or git fetch + git merge
// You will see an error message here telling you of conflicts 

// Remove the markers indicated by <<<<<<< >>>>>>> and ============

$ git commit 
$ git push // Finally push the resolved conflicts back 


