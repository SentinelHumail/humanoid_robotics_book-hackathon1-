import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Hero3D from '../components/Hero3D';
import NeuralNetwork from '../components/NeuralNetwork';

import styles from './index.module.css';
import '../css/custom.css';
import { ArrowRight, Terminal, Cpu, Brain, Check, ExternalLink, Server, School, HelpCircle, AlertTriangle } from 'lucide-react';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [isHoveringButton, setIsHoveringButton] = useState(false);

  return (
    <header className="hero-container">
      {/* Dynamic Background Elements */}
      <NeuralNetwork />
      <div className="hero-grid" />
      <div className="floating-shapes" />

      <div className="container" style={{ position: 'relative', zIndex: 10 }}>
        <div className="row" style={{ alignItems: 'center' }}>

          {/* Left Column: Text & Buttons */}
          <div className="col col--6">
            <Heading as="h1" className="hero-title">
              Digital Brain, <br />
              Physical Body.
            </Heading>
            <p className="hero__subtitle" style={{ fontSize: '20px', opacity: 0.9, maxWidth: '500px', lineHeight: '1.4', textShadow: '0 0 10px rgba(0, 243, 255, 0.3)' }}>
              The complete guide to building embodied intelligence with ROS 2, Isaac Sim, and VLA models.
            </p>

            <div className={styles.buttons} style={{ marginTop: '2rem', display: 'flex', gap: '1rem', alignItems: 'center' }}>
              <Link
                className={styles.btnStart}
                to="/docs/modules/module1"
                onMouseEnter={() => setIsHoveringButton(true)}
                onMouseLeave={() => setIsHoveringButton(false)}
              >
                START LEARNING
              </Link>
              <Link
                className={styles.btnCurriculum}
                to="#curriculum"
              >
                VIEW CURRICULUM ↓
              </Link>
            </div>
          </div>

          {/* Right Column: 3D Robot */}
          <div className="col col--6">
            <Hero3D isHoveringButton={isHoveringButton} />
          </div>

        </div>
      </div>
    </header>
  );
}

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robotics Course">

      <HomepageHeader />
      <main>
        <section id="curriculum" style={{ padding: '4rem 0' }}>
          <div className="container">
            <h2 style={{ textAlign: 'center', fontSize: '3rem', marginBottom: '3rem', textTransform: 'uppercase', letterSpacing: '4px' }}>Curriculum modules</h2>

            <div className="tabloid-grid">

              <Link to="/docs/modules/module1" className="cyber-tabloid">
                <div className="tabloid-header">
                  <h3>ROS 2 Nervous System</h3>
                  <span className="tabloid-number">01</span>
                </div>
                <div className="tabloid-content">
                  <ul>
                    <li>Middleware Architecture</li>
                    <li>Nodes & Life Cycle</li>
                    <li>DDS communication layer</li>
                    <li>Legacy bridging</li>
                  </ul>
                </div>
                <div className="tabloid-status">STATUS: ONLINE</div>
              </Link>

              <Link to="/docs/modules/module2" className="cyber-tabloid">
                <div className="tabloid-header">
                  <h3>The Digital Twin</h3>
                  <span className="tabloid-number">02</span>
                </div>
                <div className="tabloid-content">
                  <ul>
                    <li>Gazebo harmonic physics</li>
                    <li>URDF/XACRO modeling</li>
                    <li>Sensor simulation (LiDAR)</li>
                    <li>World generation</li>
                  </ul>
                </div>
                <div className="tabloid-status">STATUS: ONLINE</div>
              </Link>

              <Link to="/docs/modules/module3" className="cyber-tabloid">
                <div className="tabloid-header">
                  <h3>AI-Robot Brain</h3>
                  <span className="tabloid-number">03</span>
                </div>
                <div className="tabloid-content">
                  <ul>
                    <li>LLM Integration (LangChain)</li>
                    <li>Action Planning</li>
                    <li>Reasoning Loops</li>
                    <li>Memory Systems</li>
                  </ul>
                </div>
                <div className="tabloid-status">STATUS: LOADING...</div>
              </Link>

              <Link to="/docs/modules/module4" className="cyber-tabloid">
                <div className="tabloid-header">
                  <h3>Matrix Vision</h3>
                  <span className="tabloid-number">04</span>
                </div>
                <div className="tabloid-content">
                  <ul>
                    <li>Vision-Language-Action</li>
                    <li>RT-2 & Octo Models</li>
                    <li>Depth Perception</li>
                    <li>Spatial Awareness</li>
                  </ul>
                </div>
                <div className="tabloid-status">STATUS: LOCKED</div>
              </Link>

            </div>
          </div>
        </section>

        {/* Hardware Guide Section */}
        <section style={{ padding: '4rem 0', background: 'rgba(0,0,0,0.3)' }}>
          <div className="container">
            <h2 style={{ textAlign: 'center', fontSize: '3rem', marginBottom: '1rem', textTransform: 'uppercase', letterSpacing: '4px' }}>Hardware Requirements</h2>
            <p style={{ textAlign: 'center', maxWidth: '800px', margin: '0 auto 4rem auto', fontSize: '1.2rem', color: '#aaa' }}>
              To successfully complete the physical robotics modules, you will need access to specific hardware.
              Below is an objective comparison of your options, ranging from cloud simulation to building your own high-end lab.
            </p>

            <div className="hardware-guide-grid">

              {/* Option 1: Cloud */}
              <div className="hardware-option">
                <div className="option-header">
                  <Server size={32} color="var(--cyber-cyan)" />
                  <h3>Option 1: Cloud Simulation</h3>
                </div>
                <div className="option-tag">Budget Friendly</div>
                <p className="option-desc">Run Isaac Sim and ROS 2 in the cloud using AWS RoboMaker or specialized GPU instances.</p>
                <ul className="spec-list">
                  <li><strong>Setup:</strong> AWS / Azure / Lambda Labs</li>
                  <li><strong>Cost:</strong> ~$200/quarter (Pay-as-you-go)</li>
                  <li><strong>Pros:</strong> No hardware to buy, accessible anywhere</li>
                  <li><strong>Cons:</strong> Latency, no physical hardware experience</li>
                </ul>
                <div className="buy-links">
                  <span>Available providers:</span>
                  <a href="#" className="text-link">AWS <ExternalLink size={12} /></a>
                  <a href="#" className="text-link">Azure <ExternalLink size={12} /></a>
                </div>
              </div>

              {/* Option 2: Mid-Range */}
              <div className="hardware-option featured">
                <div className="option-header">
                  <Terminal size={32} color="var(--cyber-yellow)" />
                  <h3>Option 2: Mid-Range Station</h3>
                </div>
                <div className="option-tag recommended">Recommended</div>
                <p className="option-desc">A balanced local setup for running Sim2Real workflows and moderate model training.</p>
                <ul className="spec-list">
                  <li><strong>GPU:</strong> NVIDIA RTX 3060 (12GB) or better</li>
                  <li><strong>CPU:</strong> AMD Ryzen 7 / Intel Core i7</li>
                  <li><strong>RAM:</strong> 32GB DDR4/DDR5</li>
                  <li><strong>Est. Cost:</strong> ~$2,500</li>
                </ul>
                <div className="buy-links">
                  <span>Compare prices:</span>
                  <a href="#" className="text-link">NewEgg <ExternalLink size={12} /></a>
                  <a href="#" className="text-link">Amazon <ExternalLink size={12} /></a>
                </div>
              </div>

              {/* Option 3: High-End */}
              <div className="hardware-option">
                <div className="option-header">
                  <Cpu size={32} color="var(--cyber-purple)" />
                  <h3>Option 3: High-End Lab</h3>
                </div>
                <div className="option-tag">Max Performance</div>
                <p className="option-desc">Training VLA models and running high-fidelity photorealistic simulations locally.</p>
                <ul className="spec-list">
                  <li><strong>GPU:</strong> NVIDIA RTX 4090 (24GB)</li>
                  <li><strong>CPU:</strong> AMD Threadripper / Intel i9</li>
                  <li><strong>RAM:</strong> 128GB DDR5</li>
                  <li><strong>Est. Cost:</strong> ~$5,000+</li>
                </ul>
                <div className="buy-links">
                  <span>Part pickers:</span>
                  <a href="#" className="text-link">PCPartPicker <ExternalLink size={12} /></a>
                </div>
              </div>

              {/* Option 4: University Lab */}
              <div className="hardware-option">
                <div className="option-header">
                  <Brain size={32} color="#fff" />
                  <h3>Option 4: University Lab</h3>
                </div>
                <div className="option-tag">Free Access</div>
                <p className="option-desc">Check if your institution provides access to Robotics or AI computing clusters.</p>
                <ul className="spec-list">
                  <li><strong>Look for:</strong> CS/Robotics Dept Labs</li>
                  <li><strong>Ask for:</strong> "GPU Cluster Access"</li>
                  <li><strong>Pros:</strong> Enterprise hardware, zero cost</li>
                  <li><strong>Cons:</strong> Shared resources, queuing times</li>
                </ul>
              </div>

            </div>

            {/* Comparison Table */}
            <div className="cyber-table-container" style={{ marginTop: '4rem' }}>
              <h3 style={{ marginBottom: '1rem', color: 'var(--cyber-yellow)' }}>Feature Comparison</h3>
              <table className="cyber-table">
                <thead>
                  <tr>
                    <th>Feature</th>
                    <th>Cloud Sim</th>
                    <th>Mid-Range Local</th>
                    <th>High-End Lab</th>
                    <th>University Lab</th>
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <td><strong>Initial Cost</strong></td>
                    <td>$0 (Low)</td>
                    <td>~$2,500 (Med)</td>
                    <td>~$5,000+ (High)</td>
                    <td>$0 (Free)</td>
                  </tr>
                  <tr>
                    <td><strong>Recurring Cost</strong></td>
                    <td>High ($$$)</td>
                    <td>Low (Electricity)</td>
                    <td>Low (Electricity)</td>
                    <td>None</td>
                  </tr>
                  <tr>
                    <td><strong>Sim Performance</strong></td>
                    <td>Variable</td>
                    <td>Good (1080p)</td>
                    <td>Excellent (4K)</td>
                    <td>Enterprise</td>
                  </tr>
                  <tr>
                    <td><strong>Physical Access</strong></td>
                    <td>❌ No</td>
                    <td>✅ Yes</td>
                    <td>✅ Yes</td>
                    <td>✅ Yes (Usually)</td>
                  </tr>
                </tbody>
              </table>
            </div>

            {/* Decision Helper */}
            <div className="cyber-card" style={{ marginTop: '4rem', background: 'rgba(191, 0, 255, 0.1)', border: '1px dashed var(--cyber-yellow)' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: '1rem', marginBottom: '1rem' }}>
                <HelpCircle size={32} color="var(--cyber-yellow)" />
                <h3 style={{ margin: 0, fontSize: '1.5rem', color: '#fff' }}>Decision Helper: Which path is right for you?</h3>
              </div>
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '2rem' }}>
                <div>
                  <h4 style={{ color: 'var(--cyber-cyan)' }}>Choose <strong>Cloud</strong> if:</h4>
                  <ul>
                    <li>You have a limited upfront budget.</li>
                    <li>You move around frequently and use a laptop.</li>
                    <li>You want to start learning immediately without building a PC.</li>
                  </ul>
                </div>
                <div>
                  <h4 style={{ color: 'var(--cyber-cyan)' }}>Choose <strong>Local Station</strong> if:</h4>
                  <ul>
                    <li>You plan to work on robotics long-term.</li>
                    <li>You need to interface with real sensors/cameras (USB).</li>
                    <li>You want zero latency during teleoperation.</li>
                  </ul>
                </div>
              </div>
            </div>

          </div>
        </section>
      </main>
    </Layout>
  );
}
