---
layout: post
title:  "Automatically backup any Postgres database table into a GoogleDrive"
date:   2019-09-23 21:57:43 +0200
categories: [Linux, Python]
summary: "Learn how you can backup any Postgress database table into a GoogleDrive folder using two small scripts and the crontab."

---

In this post I will explain how to automate the process of backuping a postgres database table into a GoogleDrive cloud storage location. In my approach, we use two scripts to accomplish this: One script to produce the backup files and a second script that takes care of the uploading process. 
Note that it should be trivial to adapt this solution to automatically backup anything else into the cloud, simply change the shell script that, in this case, generate the .sql dumps of the desired postgres database table. 

If just want to get started, the [README of this project](https://github.com/frietz58/postgres_googledrive_backup) on Github contains all the relevant steps aswell ;)


<h2 id="dumping_postgres_table">Dumping a postgres table</h2>
First, we will take a look at how we can backup a postgres database table (If you wish to backup something else, you should start here). For this, we use the command `pg_dump [dbname]`, which can create script or archive dumps of any given database. I've chosen to use script dumps which are "plain-text files containing the SQL commands required to reconstruct the database to the state it was in at the time it was saved"  -- <cite><a href="https://www.postgresql.org/docs/9.3/app-pgdump.html" target="_blank">postgres documentation</a></cite>. <br>

Specifically, in the cron_backup.sh script, the line that produces the .sql dumb file is:
```
  pg_dump $target_db > $save_dir/$timestamp"_dump_"$ip4_addr".sql"
```
This create the .sql dump of `$target_db` in the folder `$save_dir`, where the name of the dump file contains the current timestamp`$timestamp` and the IPv4 adress`$ip4_addr` of the machine, on which the backup has been created. Regarding the postgres database, you need to consider that by default, not everyone user is allowed to access the database tables. If you wish to only create a backup once, right now, you could use the following command:
````
sudo -i -u postgres pg_dump $target_db > dump.sql
```
Here, `-i` runs the command as an <a href="https://www.sudo.ws/man/1.8.3/sudo.man.html#i-command" target="_blank">login shell</a> and `-u` provides the <a href="https://www.sudo.ws/man/1.8.3/sudo.man.html#u-user" target="_blank">target user</a>. When we automate this process, we use the crontab of the postgres user to take care of this, as explained <a href="#automating_via_cron">below</a>. 

<h2 id="uploading_to_googledrive">Uploading to GoogleDrive</h2>

<h3 id="enabling_v3_api">Enabling the Drive v3 API</h3>


<h2 id="automating_via_cron">Automating execution via Crontab</h2>

