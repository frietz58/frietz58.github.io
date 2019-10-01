# About this repository

This repository makes use of [Github Pages](https://pages.github.com) and [Jekyll](https://jekyllrb.com/) and serves as the backend of my blog: [A CS students notebook](https://www.compsci.blog). Thus, everything you find in this repository is what ultimately ends up on https://www.compsci.blog. 

All the posts that are visible over at [A CS students notebook](https://www.compsci.blog) exsist in the form of simple markdown files in the folder _posts and are served by Jekyll. 

Specific page layouts live in the _layouts folder, same goes for includes in the _includes folder. 

For the most part, the Jekyll theme [minima](https://github.com/jekyll/minima) is used, which is Jekylls default theme. I liked the simplistic and minimal approach so much, that I just kept on using the minima theme and simply added some css classes to extend and overwrite the basic version of the theme. 

In accordance with Jekyll's guide on [Overwriting theme defaults](https://jekyllrb.com/docs/themes/#overriding-theme-defaults), all my changes to the theme are stored in the main.scss file at /assets/main.scss. 



## Jekyll

From their own readme:
Jekyll is a simple, blog-aware, static site generator perfect for personal, project, or organization sites. Think of it like a file-based CMS, without all the complexity. Jekyll takes your content, renders Markdown and Liquid templates, and spits out a complete, static website ready to be served by Apache, Nginx or another web server. Jekyll is the engine behind [GitHub Pages](https://pages.github.com/), which you can use to host sites right from your GitHub repositories.

With Github Pages supporting / being powered by Jekyll, it is incredibly easy to setup a blog and run it on a custom domain. Which is what I did for my blog. Here come a list of links that I found usefull for getting my blog up and running:

+ [Using a custom domain with Github Pages](https://help.github.com/en/articles/about-custom-domains-and-github-pages)
+ [Jekyll's Step by Step guide](https://jekyllrb.com/docs/step-by-step/01-setup/)
+ [Jekyll's guide on overwriting theme defaults](https://jekyllrb.com/docs/themes/#overriding-theme-defaults)
+ [How to connect a GoDaddy domain with Github Pages](https://hackernoon.com/how-to-set-up-godaddy-domain-with-github-pages-a9300366c7b)

And that's about it.



## Running my blog on localhost

If you wan't to get a better Idea of how things work I encourage you to clone this repository and run my blog on localhost (given that you have Jekyll correctly installed):

```
git clone https://github.com/MiddyGoesDev/MiddyGoesDev.github.io.git finns_blog
```

```
cd finns_blog
```

```
bundle exec jekyll serve
```

