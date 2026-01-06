# Quickstart Guide

Get up and running with the Physical AI & Humanoid Robotics documentation site.

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Node.js | >= 20 | LTS recommended |
| Yarn | Latest | Package manager |
| Git | Latest | Version control |

Verify your environment:
```bash
node --version  # Should be >= 20
yarn --version  # Should be latest
```

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-org/hackathon1-book.git
   cd hackathon1-book
   ```

2. **Install dependencies**:
   ```bash
   yarn
   ```

3. **Start development server**:
   ```bash
   yarn start
   ```

   This command starts a local development server at `http://localhost:3000`. Most changes are reflected live without restarting.

## Project Structure

```
hackathon1-book/
├── docs/                      # Documentation content
│   ├── intro.md              # Homepage
│   ├── modules/              # Course modules
│   ├── hardware/             # Hardware guides
│   ├── deployment/           # Deployment options
│   └── tutorials/            # Hands-on tutorials
├── docusaurus.config.ts      # Docusaurus configuration
├── sidebars.ts               # Navigation structure
├── src/                      # Custom components
└── static/                   # Static assets
```

## Adding Content

### Creating a New Page

1. Create a Markdown file in the appropriate directory:
   ```bash
   # For modules
   touch docs/modules/new-module.md

   # For tutorials
   touch docs/tutorials/new-tutorial.md
   ```

2. Add frontmatter:
   ```markdown
   ---
   sidebar_position: 1
   title: "My New Page"
   ---
   ```

3. Add content using Markdown and MDX.

### Adding Code Examples

Use fenced code blocks with language annotations:

```python
import rclpy
from rclpy.node import Node

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.get_logger().info('Robot node initialized')
```

### Adding Images

Place images in `static/img/` and reference them:

```markdown
![Robot anatomy diagram](/img/robot-anatomy.png)
```

## Building for Production

```bash
yarn build
```

This command generates static content into the `build` directory.

## Deployment

### GitHub Pages

```bash
# Using SSH
USE_SSH=true yarn deploy

# Using GitHub username
GIT_USER=<Your GitHub username> yarn deploy
```

## Accessibility Checklist

Before committing, verify:

- [ ] All images have descriptive alt text
- [ ] Links use descriptive text (not "click here")
- [ ] Color contrast is sufficient in both themes
- [ ] Content is readable without horizontal scroll on mobile
- [ ] Interactive elements are keyboard-navigable

## Next Steps

1. Review the [Course Modules](../modules/intro.md)
2. Check [Hardware Requirements](../hardware/requirements.md)
3. Follow [ROS 2 Tutorials](../tutorials/ros2-basics.md)

## Troubleshooting

### Build fails

```bash
# Clear cache and rebuild
yarn clear
yarn build
```

### Changes not appearing

```bash
# Restart the development server
# Ctrl+C to stop, then:
yarn start
```

### Mobile layout issues

Check for:
- Tables with fixed widths
- Images without max-width
- Long code blocks without overflow wrapping
