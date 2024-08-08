# Note on repository submodule and sub-repository

- `dockerized-norlab-project-mock` is a dev utility. It is needed to implement project-deploy
  cloning/copying/checkout functionality in and out of docker container. For that reason, it is
  cloned as a full repository, not a submodule. 

<details>
  <summary style="font-weight: bolder;font-size: large;"><b> Git submodule usage notes </b></summary>

### Notes on submodule

To **clone** your repository and its submodule at the same time, use

```bash
git clone --recurse-submodules
```

Be advise, submodules are a snapshot at a specific commit of the *norlab-shell-script-tools*
repository. To **update the submodule** to its latest commit, use

```
[sudo] git submodule update --remote --recursive --init [--force]
```

Notes:

- Add the `--force` flag if you want to reset the submodule and throw away local changes to it.
  This is equivalent to performing `git checkout --force` when `cd` in the submodule root
  directory.
- Add `sudo` if you get an error such
  as `error: unable to unlink old '<name-of-a-file>': Permission denied`

To set the submodule to **point to a different branch**, use

```bash
cd <the/submodule/directory>
git checkout the_submodule_feature_branch_name
```

and use the `--recurse-submodules` flag when switching branch in your main project

```bash
cd <your/project/root>
git checkout --recurse-submodules the_feature_branch_name
```

---

### Commiting to submodule from the main project (the one where the submodule is cloned)

#### If you encounter `error: insufficient permission for adding an object to repository database ...`

```shell
# Change the `.git/objects` permissions
cd <main/project/root>/.git/objects/
chown -R $(id -un):$(id -gn) *
#       <yourname>:<yourgroup>

# Share the git repository (the submodule) with a Group
cd ../../<the/submodule/root>/
git config core.sharedRepository group
# Note: dont replace the keyword "group"
```

This should solve the problem permanently.




### References:

#### Git Submodules

- [Git Tools - Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
- [Git Submodules: Tips for JetBrains IDEs](https://www.stevestreeting.com/2022/09/20/git-submodules-tips-for-jetbrains-ides/)
- [Git submodule tutorial – from zero to hero](https://www.augmentedmind.de/2020/06/07/git-submodule-tutorial/)
- On [gitsubmodules](https://git-scm.com/docs/gitsubmodules)
#### Git command
- [git-rm](https://git-scm.com/docs/git-rm)
- [git-submodule](https://git-scm.com/docs/git-submodule#_name)

</details>
