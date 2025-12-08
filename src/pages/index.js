import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive course on ROS 2, Simulation, NVIDIA Isaac, and VLA Systems">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--3">
                <div className="text--center padding-horiz--md">
                  <h3>Module 1: ROS 2</h3>
                  <p>Master robot middleware and humanoid control</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="text--center padding-horiz--md">
                  <h3>Module 2: Simulation</h3>
                  <p>Build digital twins with Gazebo and Unity</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="text--center padding-horiz--md">
                  <h3>Module 3: Isaac</h3>
                  <p>AI-powered perception and navigation</p>
                </div>
              </div>
              <div className="col col--3">
                <div className="text--center padding-horiz--md">
                  <h3>Module 4: VLA</h3>
                  <p>Voice, LLM planning, and autonomous operation</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
