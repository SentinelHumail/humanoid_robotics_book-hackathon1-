import React from 'react';
import styles from './InteractiveElements.module.css';
import { Info, AlertTriangle, Lightbulb, Target, Zap, ChevronDown, ChevronUp } from 'lucide-react';

// Type definitions
interface CalloutProps {
  type?: 'info' | 'warning' | 'tip' | 'key' | 'danger';
  title?: string;
  children: React.ReactNode;
  icon?: React.ComponentType<any>;
}

interface DetailsProps {
  summary: string;
  children: React.ReactNode;
}

interface QuickCheckProps {
  question: string;
  answer: string;
}

interface TopicCardProps {
  title: string;
  icon?: React.ComponentType<any>;
  children: React.ReactNode;
  color?: string;
}

/**
 * Callout component with neon borders and icons
 */
export const Callout: React.FC<CalloutProps> = ({ type = 'info', title, children, icon: CustomIcon }) => {
  const configs = {
    info: { icon: Info, color: 'var(--callout-info-color)', bg: 'var(--callout-info-bg)' },
    warning: { icon: AlertTriangle, color: 'var(--callout-warning-color)', bg: 'var(--callout-warning-bg)' },
    tip: { icon: Lightbulb, color: 'var(--callout-tip-color)', bg: 'var(--callout-tip-bg)' },
    key: { icon: Target, color: 'var(--callout-key-color)', bg: 'var(--callout-key-bg)' },
    danger: { icon: Zap, color: 'var(--callout-danger-color)', bg: 'var(--callout-danger-bg)' }
  };

  const config = configs[type] || configs.info;
  const Icon = CustomIcon || config.icon;

  return (
    <div
      className={styles.callout}
      style={{
        '--callout-color': config.color,
        '--callout-bg': config.bg
      } as React.CSSProperties}
    >
      <div className={styles.calloutHeader}>
        <Icon size={20} color={config.color} className={styles.calloutIcon} />
        <span className={styles.calloutTitle}>{title || type.toUpperCase()}</span>
      </div>
      <div className={styles.calloutContent}>{children}</div>
    </div>
  );
};

/**
 * Interactive 'Learn More' section
 */
export const Details: React.FC<DetailsProps> = ({ summary, children }) => {
  const [isOpen, setIsOpen] = React.useState(false);

  return (
    <div className={`${styles.details} ${isOpen ? styles.open : ''}`}>
      <button
        className={styles.detailsSummary}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
      >
        <span className={styles.detailsIcon}>
          {isOpen ? <ChevronUp size={18} /> : <ChevronDown size={18} />}
        </span>
        {summary}
      </button>
      {isOpen && (
        <div className={styles.detailsContent}>
          {children}
        </div>
      )}
    </div>
  );
};

/**
 * Interactive check pill
 */
export const QuickCheck: React.FC<QuickCheckProps> = ({ question, answer }) => {
  const [show, setShow] = React.useState(false);

  return (
    <div className={styles.quickCheck}>
      <div className={styles.qcQuestion}>
        <span className={styles.qcTag}>QUICK CHECK</span>
        {question}
      </div>
      <button className={styles.qcToggle} onClick={() => setShow(!show)}>
        {show ? 'Hide Answer' : 'Check Answer'}
      </button>
      {show && (
        <div className={styles.qcAnswer}>
          <Zap size={16} className={styles.qcAnswerIcon} />
          {answer}
        </div>
      )}
    </div>
  );
};

/**
 * Knowledge Card
 */
export const TopicCard: React.FC<TopicCardProps> = ({ title, icon: Icon, children, color = 'var(--card-accent)' }) => (
  <div className={styles.topicCard} style={{ '--card-accent': color } as React.CSSProperties}>
    <div className={styles.topicCardHeader}>
      {Icon && <Icon size={24} className={styles.topicCardIcon} />}
      <h3>{title}</h3>
    </div>
    <div className={styles.topicCardBody}>
      {children}
    </div>
  </div>
);

/**
 * Visual Separator
 */
export const Separator = () => (
  <div className={styles.separator}>
    <div className={styles.separatorLine} />
    <div className={styles.separatorDiamond} />
    <div className={styles.separatorLine} />
  </div>
);
