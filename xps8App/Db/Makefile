TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Optimization of db files using dbst (DEFAULT: NO)
# DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += XPS8.db
DB += XPS8_arbitrary.db

DB += status_update.db
DB += status_update_arbitrary.db

#----------------------------------------------------
# Create and install (or just install)
# Archive files, templates, substitutions like this
ARCHIVE += XPS8_archive.tpl-arch
ARCHIVE += XPS8_arbitrary_archive.tpl-arch

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
REQ     += XPS8_autosave.tpl-req
REQ     += XPS8_arbitrary_autosave.tpl-req


include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

