<!--
Sync Impact Report:
- Version change: 0.0.0 -> 1.0.0
- Modified principles: All principles have been updated based on the user's input.
- Added sections: "Key Standards" and "Constraints"
- Removed sections: None
- Templates requiring updates:
    - ✅ .specify/templates/plan-template.md
    - ✅ .specify/templates/spec-template.md
    - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->

# AI/Spec-Driven Technical Book Creation using Docusaurus, Spec-Kit Plus, and Gemini Code Constitution

## Core Principles

### I. Technical Accuracy
Technical accuracy with verifiable, authoritative sources. All factual/technical claims must be validated with official documentation or reputable sources.

### II. Clarity
Clarity suitable for developers, students, and tech professionals. Use diagrams, examples, and code blocks where needed for developer clarity. Writing tone: instructional, friendly, and technically precise.

### III. Modularity
Modular, spec-driven writing aligned with Docusaurus structure. The folder and sidebar structure must follow Docusaurus best practices. Content must be delivered in Docusaurus-friendly Markdown (.mdx).

### IV. Consistency
Consistency in terminology, formatting, and style across the entire book. Citation style will be Markdown inline links with a reference list per chapter.

### V. Practical Applicability
Practical applicability with examples, code snippets, and workflows. Code samples must be tested, runnable, and consistent with the Docusaurus version used. A minimum of 30% of content should be hands-on (tutorials, examples, demos).

### VI. Reproducibility
Reproducibility of all technical steps and instructions. Ensure compatibility with GitHub Pages deployment (links, paths, file structure). All diagrams must be reproducible (Mermaid or SVG preferred).

## Key Standards
- All factual/technical claims must be validated with official documentation or reputable sources
- Citation style: Markdown inline links + reference list per chapter
- Use diagrams, examples, and code blocks where needed for developer clarity
- Code samples must be tested, runnable, and consistent with Docusaurus version used
- Writing tone: instructional, friendly, and technically precise
- Minimum 30% content should be hands-on (tutorials, examples, demos)
- Ensure compatibility with GitHub Pages deployment (links, paths, file structure)

## Constraints
- Complete book length: 20,000–40,000 words
- Each chapter: 1,000–3,000 words
- Content delivered in Docusaurus-friendly Markdown (.mdx)
- Folder + sidebar structure must follow Docusaurus best practices
- Only open-source, non-restricted resources may be quoted or used
- All diagrams must be reproducible (Mermaid or SVG preferred)

## Governance & Success Criteria
This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs and reviews must verify compliance with this constitution.

The project is successful when it meets these criteria:
- The book builds cleanly in Docusaurus with no warnings or broken links.
- It deploys successfully to GitHub Pages.
- All content is internally consistent, technically accurate, and free of plagiarism.
- Code examples compile and run without errors.
- It provides a clear learning path from beginner to practitioner level.
- The readability score is within the Flesch-Kincaid Grade 8–12 range.
- The final book meets the project’s educational and technical goals.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05