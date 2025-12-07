# Initial Docusaurus Setup Tasks

## 1. Environment Setup
- [ ] Verify Node.js (v18 or higher) and npm are installed
- [ ] Install Docusaurus CLI: `npm install -g @docusaurus/cli`
- [ ] Create project directory: `mkdir physical-ai-textbook`
- [ ] Navigate to project directory: `cd physical-ai-textbook`

## 2. Docusaurus Installation
- [ ] Initialize Docusaurus project: `npx create-docusaurus@latest website classic`
- [ ] Install additional dependencies for textbook features:
  - `npm install @docusaurus/module-type-aliases`
  - `npm install @docusaurus/types`
  - `npm install @docusaurus/preset-classic`
- [ ] Verify installation by running: `npm run start`

## 3. Basic Configuration
- [ ] Update `docusaurus.config.js` with project details:
  - Site title: "Physical AI & Humanoid Robotics"
  - Tagline: "A Complete Textbook"
  - Base URL: "/"
  - Favicon: Add textbook favicon
- [ ] Configure custom theme settings in `docusaurus.config.js`
- [ ] Set up organization and project details

## 4. Project Structure Setup
- [ ] Create docs directory structure:
  - `docs/intro.md`
  - `docs/module-1/`
  - `docs/module-2/`
  - `docs/module-3/`
  - `docs/module-4/`
- [ ] Create initial content files for each module
- [ ] Set up sidebar configuration in `sidebars.js`

## 5. Initial Content Creation
- [ ] Create introductory content in `docs/intro.md`
- [ ] Add placeholder content for Module 1 (ROS 2)
- [ ] Add placeholder content for Module 2 (Digital Twin)
- [ ] Add placeholder content for Module 3 (AI-Robot Brain)
- [ ] Add placeholder content for Module 4 (Vision-Language-Action)

## 6. Deployment Configuration
- [ ] Configure for GitHub Pages deployment in `docusaurus.config.js`
- [ ] OR configure for Vercel deployment with vercel.json
- [ ] Set up build command: `npm run build`
- [ ] Verify build process works correctly

## 7. Version Control Setup
- [ ] Initialize git repository: `git init`
- [ ] Create .gitignore file with appropriate entries
- [ ] Add initial files to git: `git add .`
- [ ] Create initial commit