import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface Stat {
  value: string;
  label: string;
  icon: string;
}

const stats: Stat[] = [
  { value: '13 Weeks', label: 'Duration', icon: 'calendar' },
  { value: '50+', label: 'Exercises', icon: 'code' },
  { value: '4 Modules', label: 'Curriculum', icon: 'book' },
  { value: 'Industry', label: 'Tools', icon: 'robot' },
];

const icons: Record<string, React.JSX.Element> = {
  calendar: (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <rect x="3" y="4" width="18" height="18" rx="2" ry="2" />
      <line x1="16" y1="2" x2="16" y2="6" />
      <line x1="8" y1="2" x2="8" y2="6" />
      <line x1="3" y1="10" x2="21" y2="10" />
    </svg>
  ),
  code: (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <polyline points="16,18 22,12 16,6" />
      <polyline points="8,6 2,12 8,18" />
    </svg>
  ),
  book: (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
      <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
    </svg>
  ),
  robot: (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <circle cx="12" cy="12" r="3" />
      <path d="M6 8v2a6 6 0 0 0 12 0V8" />
      <rect x="4" y="10" width="16" height="8" rx="2" />
      <path d="M9 18v4" />
      <path d="M15 18v4" />
    </svg>
  ),
};

const BookStats = () => {
  return (
    <div className={styles.stats}>
      {stats.map((stat, index) => (
        <div key={stat.label} className={clsx(styles.stat, `stat-${index + 1}`)}>
          <div className={styles.icon}>{icons[stat.icon]}</div>
          <div className={styles.value}>{stat.value}</div>
          <div className={styles.label}>{stat.label}</div>
        </div>
      ))}
    </div>
  );
};

export default BookStats;
