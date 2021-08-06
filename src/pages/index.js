import React from 'react';
import classnames from 'classnames';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';



function Home() {
  const context = useDocusaurusContext();
  const {siteConfig = {}} = context;
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <header className="hero hero--dark">

      {/* <header className={classnames('hero hero--dark', styles.heroBanner)}> */}

        <div className="container ">
          
          {/* <div className={classnames("row", styles.heroBanner)}> */}
          <div className="row">
            <div className={classnames("col col--4", styles.heroBanner)}>
            {/* <div className="col col--4"> */}
              {/* CSS BEM */}
              <h1 className="hero__title">{siteConfig.title}</h1> 
              <p className="hero__subtitle">{siteConfig.tagline}</p>
              
              <div className={styles.buttons}>
                <Link
                  className={classnames(
                   'button button--primary button--outline button--lg',
                   styles.getStarted,
                 )}
                  to={useBaseUrl('manuals/')}>
                  Get Started
                </Link>
              </div>
            </div>
          </div>
        </div>
      </header>
    </Layout>
  );
}

export default Home;
