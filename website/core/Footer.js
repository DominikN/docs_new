/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require('react');

class Footer extends React.Component {
  docUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    const docsUrl = this.props.config.docsUrl;
    const docsPart = `${docsUrl ? `${docsUrl}/` : ''}`;
    const langPart = `${language ? `${language}/` : ''}`;
    return `${baseUrl}${docsPart}${langPart}${doc}`;
  }

  pageUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    return baseUrl + (language ? `${language}/` : '') + doc;
  }

  render() {
    return (
      <footer className="nav-footer" id="footer">
        <section className="sitemap">
          <a href={this.props.config.baseUrl} className="nav-home">
            {this.props.config.footerIcon && (
              <img
                src={this.props.config.baseUrl + this.props.config.footerIcon}
                alt={this.props.config.title}
                width="66"
                height="58"
              />
            )}
          </a>
          <div>
            <h5>Company</h5>
            <a 
            href="https://husarion.com/about.html"
            target="_blank"
            rel="noreferrer noopener">
              About Us
            </a>
            <a 
            href="https://husarion.com/contact.html"
            target="_blank"
            rel="noreferrer noopener">
              Contact
            </a>
            <a 
              href="https://medium.com/husarion-blog"              
              target="_blank"
              rel="noreferrer noopener">
              Blog
            </a>
            <a 
              href="https://cloud.husarion.com/tos"              
              target="_blank"
              rel="noreferrer noopener">
              Terms of service
            </a>
          </div>
          <div>
            <h5>Developers</h5>
            <a
              href="#"
              target="_blank"
              rel="noreferrer noopener">
              Documentation
            </a>
            <a
              href="https://community.husarion.com/"
              target="_blank"
              rel="noreferrer noopener">
              Community forum
            </a>
            <a
              href="https://husarion.com/downloads/"
              target="_blank"
              rel="noreferrer noopener">
              Downloads
            </a>
            <a
              href="https://husarion.com"
              target="_blank"
              rel="noreferrer noopener">
              Home Page
            </a>
          </div>
          <div>
            <h5>Social</h5>
            <a 
              href="https://www.facebook.com/husarionTechnology/"              
              target="_blank"
              rel="noreferrer noopener">
              Facebook
              </a>
            <a 
              href="https://twitter.com/husarion"              
              target="_blank"
              rel="noreferrer noopener">
              Twitter
              </a>
            <a 
              href="https://github.com/husarion"              
              target="_blank"
              rel="noreferrer noopener">
              GitHub
              </a>
          </div>
        </section>
        <section className="copyright">{this.props.config.copyright}</section>
      </footer>
    );
  }
}

module.exports = Footer;
