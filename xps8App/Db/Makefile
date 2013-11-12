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
DB_INSTALLS += XPS8.db
DB_INSTALLS += XPS8_arbitrary.db

DB_INSTALLS += status_update.db
DB_INSTALLS += status_update_arbitrary.db

#----------------------------------------------------
# Create and install (or just install)
# Archive files, templates, substitutions like this
ARCHIVE += XPS8_archive.template
ARCHIVE += XPS8_arbitrary_archive.template

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
REQ     += XPS8_autosave.template
REQ     += XPS8_arbitrary_autosave.template


include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

