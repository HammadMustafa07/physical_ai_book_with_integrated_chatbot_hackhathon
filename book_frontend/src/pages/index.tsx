import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/modules/ros2-nervous-system/chapter-1-what-is-ros2">
            Start learning From Today
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics - Learn to build AI systems for embodied intelligence">
      <HomepageHeader />
      <main>
        <section className={styles.aboutSection}>
          <div className="container padding-vert--lg">
            <Heading as="h2" className={clsx('text--center', styles.sectionTitle)}>
              📘 What This Book Is About
            </Heading>
            <div className="row">
              <div className="col col--8 col--offset-2">
                <p>
                  <strong>Physical AI & Humanoid Robotics</strong> focuses on <strong>Embodied Intelligence</strong>—AI systems tightly coupled with sensors, actuators, and the physical environment.
                </p>
                <p>
                  You will learn how to:
                </p>
                <ul>
                  <li>Design robotic nervous systems using ROS 2</li>
                  <li>Create digital twins using physics-accurate simulators</li>
                  <li>Train perception and navigation models</li>
                  <li>Connect Large Language Models to robot actions</li>
                  <li>Build autonomous humanoid agents</li>
                </ul>
                <p>
                  This book bridges the gap between:
                </p>
                <ul>
                  <li>🧠 AI reasoning</li>
                  <li>🦾 Robotic motion</li>
                  <li>🌍 Real-world physics</li>
                </ul>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.structureSection}>
          <div className="container padding-vert--lg">
            <Heading as="h2" className={clsx('text--center', styles.sectionTitle)}>
              🧩 Course Structure
            </Heading>
            <div className="row">
              <div className="col col--10 col--offset-1">
                <div className="row">
                  <div className="col col--6">
                    <h3>🧠 Module 1: The Robotic Nervous System (ROS 2)</h3>
                    <ul>
                      <li>ROS 2 nodes, topics, and services</li>
                      <li>Python agents with <code>rclpy</code></li>
                      <li>URDF modeling for humanoid bodies</li>
                    </ul>
                  </div>
                  <div className="col col--6">
                    <h3>🌍 Module 2: The Digital Twin (Gazebo & Unity)</h3>
                    <ul>
                      <li>Physics, gravity, and collisions</li>
                      <li>Sensor simulation (LiDAR, cameras, IMUs)</li>
                      <li>Human-robot interaction environments</li>
                    </ul>
                  </div>
                </div>
                <div className="row">
                  <div className="col col--6">
                    <h3>👁️ Module 3: The AI-Robot Brain (NVIDIA Isaac)</h3>
                    <ul>
                      <li>Photorealistic simulation</li>
                      <li>Synthetic data generation</li>
                      <li>Visual SLAM and navigation</li>
                    </ul>
                  </div>
                  <div className="col col--6">
                    <h3>🗣️ Module 4: Vision-Language-Action (VLA)</h3>
                    <ul>
                      <li>Voice-to-Action pipelines</li>
                      <li>Natural language task planning</li>
                      <li>Autonomous humanoid agents</li>
                    </ul>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
