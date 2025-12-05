import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HeroSection() {
  return (
    <div className={styles.hero}>
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>
          Master the Future of <br />
          <span className={styles.highlight}>Generative AI</span> & <span className={styles.highlight}>Robotics</span>
        </h1>
        <p className={styles.heroSubtitle}>
          Your comprehensive guide to building intelligent systems that bridge the gap between AI theory and practical humanoid robotics applications.
        </p>
      </div>
    </div>
  );
}

function ChapterCard({ number, title, description, link, color }) {
  return (
    <Link to={link} className={styles.card} style={{ borderTopColor: color }}>
      <div className={styles.cardNumber} style={{ color }}>
        Chapter {number}
      </div>
      <h3 className={styles.cardTitle}>{title}</h3>
      <p className={styles.cardDescription}>{description}</p>
      <div className={styles.cardButton}>
        Start Reading →
      </div>
    </Link>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="Physical AI & Humanoid Robotics Textbook">
      <HeroSection />

      <main className={styles.main}>
        <div className={styles.container}>
          <h2 className={styles.sectionTitle}>Course Chapters</h2>

          <div className={styles.cardsGrid}>
            <ChapterCard
              number="1"
              title="ROS 2 Fundamentals"
              description="Master the robotic nervous system. Learn nodes, topics, services, and build your first ROS 2 robot application."
              link="/chapter-1/intro"
              color="#00D9FF"
            />

            <ChapterCard
              number="2"
              title="Gazebo Simulation"
              description="Create digital twins of robots. Learn URDF, SDF, physics engines, and sensor simulation in Gazebo."
              link="/chapter-2/intro"
              color="#FF6B6B"
            />

            <ChapterCard
              number="3"
              title="Vision-Language-Action Models"
              description="The climax! Integrate AI with robotics. Control robots with voice, vision, and LLMs. Build autonomous humanoid systems."
              link="/chapter-3/intro"
              color="#A855F7"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}
