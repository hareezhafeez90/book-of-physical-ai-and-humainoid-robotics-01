<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles:
  - PRINCIPLE_1_NAME → P1: Instructional Clarity
  - PRINCIPLE_2_NAME → P2: Practical Relevance
  - PRINCIPLE_3_NAME → P3: Technical Accuracy
  - PRINCIPLE_4_NAME → P4: Open-Source Compliance
- Added sections: Content Standards & Constraints, Development Workflow
- Removed sections: None
- Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### P1: Instructional Clarity
Content must be clear, well-structured, and designed for effective teaching and learning. All chapters, sections, and subsections must follow a logical progression that builds understanding from foundational concepts to advanced topics. Visual aids, examples, and exercises must be strategically placed to reinforce learning objectives.
<!-- Rationale: Ensures the textbook effectively serves its primary purpose as an educational resource for students -->

### P2: Practical Relevance
Emphasize real-world applications, code examples (Python/ROS preferred), and hardware case studies. Every theoretical concept must be accompanied by practical implementations, code examples, or real-world case studies that demonstrate how the concept applies in actual physical AI and humanoid robotics systems.
<!-- Rationale: Students need to understand not just the theory but how concepts translate to real implementations -->

### P3: Technical Accuracy
All formulas, concepts, and specifications must be rigorously correct and current. All technical content must be verified against peer-reviewed articles, industry standards, or primary documentation. Mathematical derivations, algorithm implementations, and hardware specifications must be precise and error-free.
<!-- Rationale: As an academic textbook, technical accuracy is fundamental to credibility and educational value -->

### P4: Open-Source Compliance
All text, code, and diagrams must be suitable for open-source distribution (e.g., CC BY-NC-SA). All code examples must be license-compatible for open-source distribution, and all third-party content must have appropriate attribution and permissions for inclusion in an open-source textbook.
<!-- Rationale: Enables collaborative development and ensures legal compliance with open-source distribution -->

### P5: Docusaurus-First Architecture
All content must be structured and formatted for Docusaurus deployment to GitHub Pages. Content organization, navigation, and styling must align with Docusaurus requirements and best practices for web-based documentation.
<!-- Rationale: Ensures consistent deployment and maintainable content management -->

### P6: Academic Rigor
Minimum 50% of concepts/algorithms must be verified against peer-reviewed articles, industry standards, or primary documentation. All significant claims must be supported by verifiable sources, and all code examples must be syntax-checked and runnable where applicable.
<!-- Rationale: Maintains the academic integrity and scholarly value of the textbook -->

## Content Standards & Constraints

**Platform**: Must be built and managed using Docusaurus.
**Content Scope**: Minimum of 5-7 core modules/chapters covering essential Physical AI & Humanoid Robotics topics.
**Word Count**: Target 15,000–25,000 words total for the initial draft.
**Writing Style**: Professional, academic tone suitable for a textbook with clear, well-structured content.
**Code Standards**: All conceptual code examples must be provided, syntax-checked, and runnable (where applicable) using Python/ROS preferred technologies.

## Development Workflow

**Development Tool**: Primary content generation driven by Spec-Kit Plus and Claude Code.
**Review Process**: All technical concepts must be fact-checked and verified for accuracy before merging.
**Quality Gates**: All content must pass Docusaurus Markdown and configuration checks before deployment.
**Target Audience**: Undergraduate/Graduate students in Computer Science and Engineering with foundational understanding of AI/Robotics principles.

## Governance

This constitution supersedes all other development practices for this project. All changes to the textbook content must comply with these principles. Amendments to this constitution require documentation of the change, approval from project maintainers, and a migration plan for existing content if needed. All PRs and reviews must verify compliance with these principles before approval.

**Version**: 1.1.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
