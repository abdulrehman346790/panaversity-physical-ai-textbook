import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title fade-in">{siteConfig.title}</h1>
        <p className="hero__subtitle fade-in">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            📚 Start Learning
          </Link>
          <Link
            className="button button--primary button--lg"
            to="https://github.com/abdulrehman346790/panaversity-physical-ai-textbook"
            style={{ marginLeft: '1rem' }}>
            ⭐ Star on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

const FeatureList = [
  {
    title: '🤖 AI-Powered Chatbot',
    description: (
      <>
        Ask questions and get instant answers from our intelligent RAG chatbot,
        powered by Gemini 2.5 Flash and trained on the entire textbook content.
      </>
    ),
  },
  {
    title: '📚 Comprehensive Curriculum',
    description: (
      <>
        Learn Physical AI from fundamentals to advanced topics including ROS 2,
        robot simulation, vision-language-action models, and deployment strategies.
      </>
    ),
  },
  {
    title: '🔧 Hands-on Projects',
    description: (
      <>
        Build real-world projects with step-by-step tutorials, code examples,
        and practical exercises designed for beginners and professionals alike.
      </>
    ),
  },
  {
    title: '🚀 Industry-Ready Skills',
    description: (
      <>
        Master the tools and technologies used by leading robotics companies
        including ROS 2, Webots, Gazebo, and modern AI frameworks.
      </>
    ),
  },
  {
    title: '🎓 Beginner-Friendly',
    description: (
      <>
        No prior robotics experience required. Start from the basics and
        progress to advanced concepts with clear explanations and visual diagrams.
      </>
    ),
  },
  {
    title: '🌐 Open Source',
    description: (
      <>
        Completely free and open source. Contribute, suggest improvements,
        and help build the future of Physical AI education.
      </>
    ),
  },
];

function Feature({ title, description }) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="card" style={{ height: '100%', padding: '1.5rem' }}>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className={styles.statCard}>
              <h2>3+</h2>
              <p>Comprehensive Chapters</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.statCard}>
              <h2>224</h2>
              <p>Knowledge Chunks Indexed</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.statCard}>
              <h2>100%</h2>
              <p>Free & Open Source</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <h2>Ready to Build the Future of Robotics?</h2>
        <p>Start your journey into Physical AI today</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started Now →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Learn Physical AI - From fundamentals to deployment. Master ROS 2, robot simulation, and AI-powered robotics.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <StatsSection />
        <CTASection />
      </main>
    </Layout>
  );
}
