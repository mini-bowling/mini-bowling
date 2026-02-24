# Git Beginner's Guide

A quick-start guide for new contributors to the mini-bowling project.

## 1. Setting Up Your GitHub Account

Before touching any code, you need a home for your work.

1. Go to [GitHub.com](https://github.com) and sign up.
2. Use a professional username — it often becomes your developer portfolio.

## 2. Installing and Connecting Fork

[Fork](https://git-fork.com) is a GUI (Graphical User Interface) that makes Git's commands visual and easy to manage. There are other GUIs available such as Sourcetree or GitKraken, and Git can also be used from the CLI as well. For simplicity, we are detailing how to use Fork in this document.

1. Download and install Fork from [git-fork.com](https://git-fork.com).
2. Open Fork > **File** > **Accounts**.
3. Click **+** and select **GitHub**.
4. Log in using the **OAuth** method. This links Fork to your GitHub account so you don't need to type your password on every push.

## 3. Cloning the Repository

"Cloning" downloads a local copy of the project to your computer.

1. On the GitHub project page, click the green **<> Code** button and copy the URL.
2. In Fork, go to **File** > **Clone**.
3. Paste the URL — Fork will auto-fill the name. Choose a local folder and click **Clone**.

## 4. Understanding Branches & Fetching

Think of the `main` branch as the finished, working project. You never work directly on it — instead, you create a **branch** as a safe sandbox.

### Fetching

Before starting any work, check whether others have made changes:

- **Fetch** tells Fork to check the server for updates. It does not change your local files — it just shows you what's new. Note that Fork will fetch updates from GitHub automatically on a timer basis (e.g. every 10 minutes).
- **Rule of thumb:** Always fetch before starting a new task.

### Creating a Branch

1. Click the **Branch** button in the top toolbar.
2. Name it something descriptive, like `add-pinsetter-logic` or `fix-ball-return-model`.
3. Click **Create and Checkout**. You are now working in your own sandbox.

## 5. Saving and Pushing Your Work

Once you've edited a file (like a `.ino` sketch), Fork will show **Uncommitted Changes**.

1. **Stage** — Check the box next to the files you want to save.
2. **Commit** — Write a short, clear message (e.g., `Adjusted servo timing for pin reset`). This saves the changes to your local history.
3. **Push** — Click the **Push** button. This sends your commit up to GitHub.

## 6. Creating a Pull Request (PR)

A Pull Request asks the team to review and merge your changes into the main project.

1. Go to the project page on GitHub.
2. You'll usually see a banner saying **"Your branch had recent pushes."** Click **Compare & pull request**.
3. Describe what you changed and why.
4. Click **Create pull request**.

Someone else can then review your changes before they officially become part of the project.
