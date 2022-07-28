# How to use git without installing `gitCredentialManager` on Jetson Xavier AGX 

`gitCredentialManager` isn't supported for linux arm64 devices. 

Therefore, we have to manually set our ssh-keys. 

Follow this instruction:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys


You might have to do this for your specific git repo.
1. Navigate to your repo
2. Set the remote manually

```bash
git remote set-url origin git@github.com:username/your-repository.git
```

You might also need to configure your local git client 

```bash
git config --global user.name "your_github_username"
$ git config --global user.email "your_github_email"
```