# Deployment Guide - Physical AI & Humanoid Robotics Course

## Module 1 Status: ✅ Ready for Deployment

### Quick Start - Local Testing

Test the site locally before deploying:

```bash
# Start local development server
npm run serve

# The site will be available at:
# http://localhost:3000/humanoid_aibook/
```

Expected output:
- Course homepage with Module 1 navigation
- All 3 chapters accessible
- Code syntax highlighting working
- Navigation between chapters functional

---

## GitHub Pages Deployment

### Prerequisites

1. **GitHub Repository**: Ensure you have a GitHub repository created
2. **GitHub Pages Enabled**: Enable GitHub Pages in repository settings
3. **Git Configured**: Local git should be configured with your credentials

### Configuration Check

The following settings are already configured in `docusaurus.config.js`:

```javascript
url: 'https://your-username.github.io',
baseUrl: '/humanoid_aibook/',
organizationName: 'your-username',
projectName: 'humanoid_aibook',
deploymentBranch: 'gh-pages',
```

**⚠️ IMPORTANT**: Update `your-username` to your actual GitHub username before deploying.

### Deployment Steps

#### Option 1: Automated Deployment (Recommended)

The GitHub Actions workflow is already configured at `.github/workflows/deploy.yml`.

**Steps**:

1. **Update Repository URL** in `docusaurus.config.js`:
   ```javascript
   organizationName: 'YOUR-GITHUB-USERNAME',
   ```

2. **Commit and Push**:
   ```bash
   git add .
   git commit -m "Module 1 complete - ready for deployment"
   git push origin master
   ```

3. **Automatic Build**: GitHub Actions will automatically:
   - Run `npm run build`
   - Deploy to `gh-pages` branch
   - Make site live at `https://YOUR-USERNAME.github.io/humanoid_aibook/`

4. **Verify Deployment**:
   - Check Actions tab in GitHub for build status
   - Visit your site URL after build completes

#### Option 2: Manual Deployment

If you prefer manual deployment:

```bash
# Install GitHub Pages deployment tool (if not already installed)
npm install --save-dev gh-pages

# Deploy to GitHub Pages
npm run deploy
```

This will:
- Build the static site
- Push to `gh-pages` branch
- Trigger GitHub Pages rebuild

---

## Post-Deployment Verification

After deployment, verify the following:

### ✅ Checklist

- [ ] Site loads at `https://YOUR-USERNAME.github.io/humanoid_aibook/`
- [ ] Homepage displays correctly with course introduction
- [ ] Module 1 navigation visible in sidebar
- [ ] Chapter 1.1 (ROS 2 Fundamentals) accessible
- [ ] Chapter 1.2 (ROS 2 for Humanoids) accessible
- [ ] Chapter 1.3 (URDF for Humanoids) accessible
- [ ] Code blocks have syntax highlighting
- [ ] Navigation between chapters works
- [ ] Search functionality operational (if enabled)
- [ ] Mobile responsive design working

### Common Issues and Solutions

| Issue | Solution |
|-------|----------|
| 404 errors on all pages | Check `baseUrl` in config matches repo name |
| CSS not loading | Verify `url` in config uses HTTPS |
| Images broken | Ensure images are in `static/img/` directory |
| Links broken | Use relative paths, not absolute |
| Build fails on GitHub | Check Node.js version in workflow matches local |

---

## Content Quality Review

Before public deployment, review:

### Chapter 1.1: ROS 2 Fundamentals
- [x] Learning objectives clear
- [x] Code examples tested
- [x] Troubleshooting sections included
- [x] Links to external resources working
- [x] Estimated time accurate (90 min)

### Chapter 1.2: ROS 2 for Humanoids
- [x] Action server examples complete
- [x] IMU integration explained
- [x] Hands-on project included
- [x] Challenge exercises provided
- [x] Estimated time accurate (120 min)

### Chapter 1.3: URDF for Humanoids
- [x] URDF syntax explained
- [x] Xacro usage demonstrated
- [x] Both simple and complex examples provided
- [x] Visualization instructions included
- [x] Estimated time accurate (110 min)

---

## Feedback Collection

Once deployed, collect feedback through:

1. **GitHub Issues**: Enable issues for bug reports and suggestions
2. **Discussion Forum**: Consider enabling GitHub Discussions
3. **Analytics**: Add Google Analytics (optional) to track usage

### Suggested Issue Templates

Create `.github/ISSUE_TEMPLATE/` with:
- `bug_report.md` - For technical issues
- `content_feedback.md` - For content improvements
- `feature_request.md` - For new module suggestions

---

## Maintenance Schedule

After deployment, plan for:

- **Weekly**: Monitor GitHub Issues
- **Monthly**: Update dependencies (`npm update`)
- **Quarterly**: Review and update content based on ROS 2 releases
- **As needed**: Fix broken links, update code examples

---

## Next Module Preview

While Module 1 is live, begin planning Module 2:

- **Module 2**: Simulation Environments (Gazebo & Unity)
- **Target**: 3 chapters, similar structure
- **Timeline**: Estimate 2-3 weeks for content generation
- **Prerequisites**: Module 1 feedback incorporated

---

## Rollback Plan

If critical issues are found post-deployment:

```bash
# Revert to previous commit
git revert HEAD
git push origin master

# Or rollback gh-pages branch
git checkout gh-pages
git reset --hard <previous-commit-hash>
git push --force origin gh-pages
```

---

## Support Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **GitHub Pages**: https://docs.github.com/en/pages
- **ROS 2 Docs**: https://docs.ros.org/en/humble/
- **Project Issues**: https://github.com/YOUR-USERNAME/humanoid_aibook/issues

---

## Success Metrics

Track these metrics post-deployment:

- **Page Views**: Total and per chapter
- **Average Time on Page**: Should align with estimated times (90-120 min)
- **Bounce Rate**: Target <40% for educational content
- **GitHub Stars**: Indicator of community interest
- **Issues/Feedback**: Quality and quantity of student feedback

---

**Last Updated**: 2025-12-08
**Module 1 Status**: ✅ Production Ready
**Build Status**: ✅ Passing (35/35 tests)
