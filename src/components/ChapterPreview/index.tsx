import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface Chapter {
  number: string;
  title: string;
  description: string;
  duration: string;
}

const chapters: Chapter[] = [
  {
    number: '01',
    title: 'The Robotic Nervous System',
    description: 'ROS 2 fundamentals - nodes, topics, services, and URDF models that power modern robot architectures.',
    duration: 'Week 1-3',
  },
  {
    number: '02',
    title: 'The Digital Twin',
    description: 'Master Gazebo physics simulation and create realistic robot environments before touching hardware.',
    duration: 'Week 4-6',
  },
  {
    number: '03',
    title: 'The AI-Robot Brain',
    description: 'Deploy NVIDIA Isaac ROS for perception, VSLAM, and intelligent decision making.',
    duration: 'Week 7-10',
  },
  {
    number: '04',
    title: 'Vision-Language-Action',
    description: 'Build VLA models and voice-controlled robots with your capstone humanoid project.',
    duration: 'Week 11-13',
  },
];

const ChapterPreview = () => {
  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <span className={styles.chapterLabel}>Course Curriculum</span>
        <h2 className={styles.title}>What You'll Learn</h2>
        <p className={styles.subtitle}>
          A comprehensive journey from ROS 2 fundamentals to building autonomous humanoid robots
        </p>
      </div>

      <div className={styles.chapters}>
        {chapters.map((chapter, index) => (
          <Link
            key={chapter.number}
            to={`/docs/modules/module${index + 1}-${index === 0 ? 'ros2' : index === 1 ? 'gazebo' : index === 2 ? 'isaac' : 'vla'}`}
            className={clsx(styles.chapter, `chapter-${index + 1}`)}
          >
            <div className={styles.chapterNumber}>
              <span className={styles.number}>{chapter.number}</span>
              <div className={styles.chapterLine} />
            </div>
            <div className={styles.chapterContent}>
              <div className={styles.chapterMeta}>
                <span className={styles.duration}>{chapter.duration}</span>
              </div>
              <h3 className={styles.chapterTitle}>{chapter.title}</h3>
              <p className={styles.chapterDescription}>{chapter.description}</p>
              <span className={styles.readMore}>
                Read Chapter
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M5 12h14M12 5l7 7-7 7" />
                </svg>
              </span>
            </div>
          </Link>
        ))}
      </div>
    </div>
  );
};

export default ChapterPreview;
