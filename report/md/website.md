# About this website

This report was written in a variant of [`Markdown`](https://daringfireball.net/projects/markdown/syntax) (MD), a lightweight markup language that can be easily read without an interpreter. We used the [`typora`](https://typora.io) editor for writting.

The website also incorporates code blocks with syntax highlighting using [`prism`](http://prismjs.com); the CSS style is a slightly modified version of [`Pixyll`](https://github.com/johnotander/pixyll); the math formulae use [`MathJax`](https://www.mathjax.org) and the overall document conversion is made via [`pandoc`](http://pandoc.org); and citations use [`Bibcite`](https://github.com/bacor/bibcite) coupled with _BibTex_.

The 3D vector fields are drawn using a custom script that leverages the [`plotly`](https://plot.ly) library.

The webserver uses [`node`](https://nodejs.org/en/) and the [`express`](http://expressjs.com) webserver framework for routing, and a few custom `bash` scripts have been written to generate the files automatically.



In the open-source spirit, the code for this report is available on the GitHub repository of the project.