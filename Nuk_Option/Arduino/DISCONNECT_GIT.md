# How to Disconnect Libraries Folder from GitHub

The `libraries` folder at `/Users/emmettstralka/Documents/GitHub/E80-Team15/libraries/` is part of a Git repository. Here's how to disconnect it:

## Option 1: Remove from Git Tracking (Recommended)

This keeps the folder but stops Git from tracking it:

```bash
cd /Users/emmettstralka/Documents/GitHub/E80-Team15/
git rm -r --cached libraries/
echo "libraries/" >> .gitignore
git commit -m "Remove libraries folder from Git tracking"
```

## Option 2: Add to .gitignore (If Not Already Tracked)

If the libraries folder isn't heavily tracked, just add it to .gitignore:

```bash
cd /Users/emmettstralka/Documents/GitHub/E80-Team15/
echo "libraries/" >> .gitignore
git add .gitignore
git commit -m "Ignore libraries folder"
```

## Option 3: Move Libraries Folder Outside Git Repository

Move the libraries to a location outside the Git repo:

```bash
# Move libraries folder outside Git repo
mv /Users/emmettstralka/Documents/GitHub/E80-Team15/libraries/ ~/Documents/Arduino/libraries_from_github/

# Then remove from Git
cd /Users/emmettstralka/Documents/GitHub/E80-Team15/
git rm -r libraries/
git commit -m "Move libraries folder outside repository"
```

## Quick Command (Option 1 - Recommended)

Run this to disconnect the libraries folder:

```bash
cd /Users/emmettstralka/Documents/GitHub/E80-Team15/
git rm -r --cached libraries/ 2>/dev/null || echo "Not tracked"
echo "libraries/" >> .gitignore
git add .gitignore
```

**Note:** This will stop Git from tracking the libraries folder, but won't delete the actual folder or its contents.

