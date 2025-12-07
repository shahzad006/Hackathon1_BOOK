---
id: 8
title: Fix Vercel deployment docusaurus command not found
stage: misc
date_iso: 2025-12-07
surface: agent
model: gemini-1.5-flash
feature: create-robotics-book
branch: 
user: 
command: 
labels: ["vercel", "deployment", "docusaurus", "fix"]
links: 
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: 
  - vercel.json
tests_yaml: 
prompt_text: |-
  13:38:23.447 Running build in Portland, USA (West) – pdx1
  13:38:23.448 Build machine configuration: 2 cores, 8 GB
  13:38:23.547 Cloning github.com/shahzad006/Hackathon1_BOOK (Branch: main, Commit: 14b5185)
  13:38:23.547 Previous build caches not available.
  13:38:23.948 Cloning completed: 401.000ms
  13:38:24.321 Running "vercel build"
  13:38:25.106 Vercel CLI 49.0.0
  13:38:25.768 Installing dependencies...
  13:38:26.712 
  13:38:26.713 added 1 package in 710ms
  13:38:26.743 sh: line 1: docusaurus: command not found
  13:38:26.747 Error: Command "docusaurus build" exited with 127              When i deploy on vercel me show this error please fixed
response_text: |-
  I have created the `vercel.json` file to address the Vercel deployment error. This configuration explicitly instructs Vercel to use `npm run build` within the `docusaurus` directory for the build process, which should resolve the "docusaurus: command not found" issue.

  **Acceptance Criteria:**
  *   A `vercel.json` file is present in the project root.
  *   The `vercel.json` defines `installCommand`, `buildCommand`, and `outputDirectory` for the Docusaurus project.
  *   The `vercel.json` includes `routes` and `rewrites` to correctly serve the Docusaurus build output.

  **Follow-ups and Risks:**
  *   **Follow-up:** Please redeploy your project on Vercel to confirm the resolution of the error.
  *   **Risk:** If Vercel's project-specific settings are configured to override the `vercel.json` file, you may need to adjust those settings directly within the Vercel dashboard.
outcome: 
evaluation: 
---