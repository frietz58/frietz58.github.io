---
title: "Easy language learning with ChatGPT and Python "
last_modified_at: 2023-09-10 14:46:12 +0200
categories:
  - Machine Learning
tags:
  - Python
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "This post describes how to use ChatGPT with python for easy language (or anything else) learning."
---

<h2 id="motivation">Motivation and introduction</h2>
ChatGPT is everywhere, has disruptive potential, and can do many great things, given the right instructions (prompts). 
ChatGPT or other GPT models tend to perform well when the conversation is centered around a topic that is abundantly covered in their training data, just like most ML algorithms perform well when the test data is <i>similar enough</i> to the training data.
As such, it is no surprise that ChatGPT can generate code for almost any programming language, with many blogs and documentation being publicly available. 
Although ChatGPT might struggle with generating correct code for sophisticated and complex programs, it knows about data structures and basic I/O stuff, so enough for small scripts.
This makes it very easy to combine ChatGPT's generative capabilities in virtually any topic with simple programs, which can be useful for many small projects.

To give one example of such a project, in the following, I show how to generate a deck of flashcards with the most useful, basic words in any language using ChatGPT and a few lines of Python code.
The technical aspect of this is relatively trivial, however, I want to provide easy-to-replicate steps even for people that have no background in computer science.
<br>


<h2 id="chatgpt_language_tutor">ChatGPT as a language tutor</h2>
Since I am doing my Ph.D. in Sweden, without being a native speaker of any Scandinavian  language, I am always looking for opportunities to practice Swedish. 
Since ChatGPT is capable of communicating in almost every language, it can also generate lists of vocabularies in any language. 
For example, one can ask it to generate the 100 most useful words for a particular language, as well as the translation of those words into any other language. 
One can further ask it to also provide an example sentence for each word, as well as the translation of the example sentence. 
There are almost no limits here, as long as we provide the right prompt, ChatGPT will provide the corresponding result (which is sometimes better, sometimes worse).
<br>

For my particular use case with Swedish-English translations, I found ChatGPT to provide okay to decent translations.
One should of course take everything that a GPT model generates with a grain of salt, but given my rudimental understanding of Swedish, I deemed the output of sufficient quality to use for some additional studying.
Give a vocabulary containing whatever we want to learn, the next step is to get ChatGPT to provide it in a data structure that is convenient to work with programmatically.
In my case, I wanted to have the data stored in a list of Python dictionaries.
Given clear instructions for how the list of dictionaries should be populated, ChatGPT will generate data that we can manipulate and make use of programmatically.

[Here is a short demonstration of the ChatGPT conversation](https://chat.openai.com/share/b5e1acf1-4233-4b35-a7f6-e02b9c499863) that I used to get a vocabulary of Swedish words and example sentences, as well as their English translation, stored in a list of Python dictionaries.

<h2 id="python_flashcards">Creating flashcards with Python</h2>
Given that we now have a list of Python dictionaries containing things we want to memorize, we can use the great, open-source tool [Anki](https://apps.ankiweb.net/) and the corresponding [Python API](https://github.com/kerrickstaley/genanki) to automatically create flash cards.
Anki is amazing, it even has  a mobile app that can synchronize sets of flashcards across multiple devices.
I have used it extensively throughout my university studies and can not recommend it enough.

[Genanki](https://github.com/kerrickstaley/genanki), at the same time, makes it trivial to generate Anki decks using Python. 
Thus, all that is left to do is load the ChatGPT vocabulary into python and create the flashcards using genanki.
The GitHub repository provides a great example of how to do this, however, for completeness, here is the full script that I used:
```python
import genanki
import random

word_list = [
    {
        'swe_word': 'att',
        'eng_word': 'to',
        'swe_sentence': 'Jag vill att du ska göra det.',
        'eng_sentence': 'I want you to do it.',
    },
    # replace this with your own data...
]


if __name__ == "__main__":
    random_model_id = random.randrange(1 << 30, 1 << 31)
    random_deck_id = random.randrange(1 << 30, 1 << 31)

    # genanki.Model defines the template for each flash card in our deck
    my_model = genanki.Model(
        random_model_id,  # needs to be unique
        'SWE & ENG Language learning model',
        fields=[
          {'name': 'swe_word'}, 
          {'name': 'eng_word'}, 
          {'name': 'swe_sentence'}, 
          {'name': 'eng_sentence'},
        ],
        templates=[
          {
            'name': 'Card 1',
            'qfmt': 'What is the meaning of "<i></i>"?<br><br>Example: <i></i>',
            'afmt': '<hr id="answer">' + \
                    'The meaning of <i>""</i> is <i>""</i>.' + \
                    '<br><br>Example: <i></i>',
            },
        ])

    my_deck = genanki.Deck(
        random_model_id,  # needs to be unique
        'Swedish & English Language learning'
    )

    for card in word_list:
        my_note = genanki.Note(
            model=my_model,
            fields=[card['swe_word'], card['eng_word'], card['swe_sentence'], card['eng_sentence']])

        my_deck.add_note(my_note)

    genanki.Package(my_deck).write_to_file('swe_eng.apkg')

``` 
As can be seen, the script simply iterates over the vocabulary, creates a flashcard for each dictionary in the list, and adds the card to the deck.
The deck is then saved as a file and can be imported into Anki.

This setup relies on manually copying the ChatGPT data into a Python file, which is a minor annoyance for perfectionistic programmers... 
Of course, this could be avoided by making use of ChatGPT's API, but unfortunately, I haven't gotten access yet, even after paying for ChatGPT premium, solely to gain API access. Oh well ¯\\_(ツ)_/¯...


The resulting deck, diplayed in the AnkiDroid app, looks as follows:
<figure>
  <img src="/assets/img/genanki/anki.jpg">
  <figcaption>Example flashcard with ChatGPT generated content displayed in the AnkiDroid app. The deck is created with the above give code and synchronized to the mobile device via AnkiWeb.</figcaption>
</figure>

<h2 id="summary">Summary and conclusion</h2>
So, in summary, ChatGPT can be used to create vocabularies with translation between any language. 
These vocabularies can have arbitrary auxiliary  information, like example sentences or different word forms. 
ChatGPT can then be asked to output these vocabularies in convenient data structures, like Python dictionaries. 

The open-source tool Anki for flashcard learning can be used to study these vocabularies. 
The easy-to-use Python package genanki makes it trivial to generate anki decks programmatically using Python, for example using the above-given script.

And that's it, I hope you found this post interesting and perhaps useful for your own studies :)

Cheers,<br>
*Finn*.
<br>
