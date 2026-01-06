# Research: Physical AI & Humanoid Robotics Documentation Site

This document captures research findings for building the documentation site.

## Docusaurus Configuration

### Technology Stack Decision

**Chosen**: Docusaurus 3.9.2 with TypeScript

**Rationale**:
- Existing project already uses Docusaurus 3.9.2, React 19, TypeScript 5.6
- Built-in docs, blog, and pages cover all requirements
- MDX support enables interactive tutorials with code examples
- Strong TypeScript integration for type-safe configuration
- Large community and active maintenance
- GitHub Pages deployment built-in

**Alternatives Considered**:
- **GitBook**: More restrictive, paid features for private docs, less customization
- **MkDocs**: Python-based (different skill set), less interactive, good for simple docs
- **Custom React app**: Overkill for documentation, higher maintenance burden

### Project Type Classification

**Type**: Static documentation site

**Characteristics**:
- Content stored as Markdown/MDX files
- Build process generates static HTML
- No backend server required
- GitHub Pages for hosting
- Content-first approach aligns with constitution Principle I

## Documentation Structure

### Navigation Architecture

**Decision**: Hierarchical sidebar with logical content grouping

**Structure**:
```
Getting Started
├── Introduction (Course overview)
├── Hardware Requirements
└── Setup Guides

Course Content
├── Module 1: ROS 2
├── Module 2: Gazebo
├── Module 3: NVIDIA Isaac
└── Module 4: VLA

Resources
├── Tutorials
├── Assessments
└── Deployment Options
```

**Rationale**:
- Mirrors course module structure (4 modules)
- Hardware section separated for easy reference during setup
- Tutorials section enables hands-on learning flow
- Deployment options comparison accessible from main nav

### Content Organization

**Decision**: Use Docusaurus doc folders with sidebars.ts

**Implementation**:
- docs/intro.md - Homepage
- docs/modules/*.md - Course modules
- docs/hardware/*.md - Hardware requirements
- docs/deployment/*.md - Deployment options
- docs/tutorials/*.md - Hands-on tutorials

## Code Example Handling

### Syntax Highlighting

**Decision**: Use Docusaurus CodeBlock with Prism

**Languages to support**:
- Python (ROS 2 rclpy examples, Isaac algorithms)
- YAML (configuration files, URDF/SDF)
- Bash (installation commands)
- Markdown (file format examples)

### Code Block Features

- Language annotations (```python, ```yaml, ```bash)
- Line numbering for tutorials
- Title annotations for file names
- Copy-paste button (built-in)
- Line highlighting for emphasis

## Accessibility Implementation

### WCAG 2.1 AA Compliance

**Required by**: Constitution Principle V (Accessible Content)

**Implementation approach**:
1. **Alt text**: All images require descriptive alt attributes
2. **Heading hierarchy**: Proper h1→h2→h3 structure
3. **Keyboard navigation**: Test all interactive elements
4. **Color contrast**: 4.5:1 minimum ratio
5. **ARIA labels**: Where needed for custom components

**Docusaurus built-in accessibility**:
- Skip-to-content links
- Semantic HTML
- Focus management
- Dark mode color optimization

### Mobile Responsiveness

**Decision**: Responsive design with mobile-first approach

**Considerations**:
- Hamburger menu for mobile nav
- Readable text without horizontal scroll
- Touch-friendly interactive elements
- Responsive tables for hardware specs

## Deployment Strategy

### GitHub Pages

**Decision**: Use GitHub Pages for hosting

**Rationale**:
- Already configured in existing docusaurus.config.ts
- Free hosting for public repositories
- Automatic deployment on main branch
- Custom domain support

**Configuration**:
```typescript
// docusaurus.config.ts
organizationName: 'your-org', // or GitHub username
projectName: 'hackathon1-book',
```

### Build Verification

**Required by**: Constitution Principle III (Build Verification)

**Process**:
1. Local: `yarn build` must succeed before commit
2. CI: GitHub Actions runs build on every PR
3. Deploy: Only merged code deploys to production

## Content Migration Strategy

### Course Content Mapping

From the provided course material:

| Week Range | Module | Content Type |
|------------|--------|--------------|
| 1-2 | Introduction | Overview, concepts |
| 3-5 | ROS 2 | Technical tutorials |
| 6-7 | Gazebo | Simulation guides |
| 8-10 | NVIDIA Isaac | AI/ML tutorials |
| 11-12 | Humanoid Development | Advanced topics |
| 13 | Conversational Robotics | Integration guide |

### Hardware Documentation

**Sections needed**:
1. Digital Twin Workstation requirements
2. Edge Kit setup (Jetson Orin Nano)
3. Robot Lab options comparison
4. Sim-to-Real architecture diagram

### Deployment Options

**Comparison required**:
- On-Premise (physical lab)
- Cloud-Native (AWS/Azure)
- Hybrid approach

## Research Summary

All technical decisions align with the constitution:
- Content-first documentation approach
- Consistent Docusaurus styling
- Build verification before deployment
- Version management via GitHub Pages
- Accessible content requirements met

**No unresolved technical decisions** - proceeding to implementation planning.
