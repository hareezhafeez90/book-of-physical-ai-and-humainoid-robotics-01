# Physical AI & Humanoid Robotics Textbook

This repository contains an open-source textbook on Physical AI and Humanoid Robotics.

## GitHub Pages Deployment

This site is configured for deployment to GitHub Pages. The deployment happens automatically via GitHub Actions when changes are pushed to the main branch.

### To Enable GitHub Pages Deployment:

1. Make sure the `gh-pages` branch exists (it will be created by the GitHub Action)
2. Go to your repository settings → Pages
3. Select "Deploy from a branch"
4. Choose `gh-pages` branch and `/ (root)` folder
5. Click "Save"

### Manual Deployment

If you prefer to deploy manually:

1. Build the site: `npm run build`
2. The build output is in the `build/` directory
3. Deploy the contents of the `build/` directory to your GitHub Pages branch

### Base URL Configuration

The site is configured with a base URL of `/book-of-physical-ai-and-humainoid-robotics-01/` to match GitHub Pages subdirectory deployment. If you're deploying to a custom domain, you may need to adjust the `baseUrl` in `docusaurus.config.js`.