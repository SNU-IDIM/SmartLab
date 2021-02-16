# This is an auto-generated Django model module.
# You'll have to do the following manually to clean this up:
#   * Rearrange models' order
#   * Make sure each model has one field with primary_key=True
#   * Make sure each ForeignKey has `on_delete` set to the desired behavior.
#   * Remove `managed = False` lines if you wish to allow Django to create, modify, and delete the table
# Feel free to rename the models, but don't rename db_table values or field names.
from django.db import models


class Printer(models.Model):
    status = models.TextField(blank=True, null=True)
    recent_work = models.TextField(blank=True, null=True)
    nozzle_temperature = models.FloatField(blank=True, null=True)
    time_left = models.TextField(blank=True, null=True)
    time_elapsed = models.TextField(blank=True, null=True)
    subject_name = models.TextField(blank=True, null=True)
    device_name = models.TextField(blank=True, null=True)
    ip_port = models.TextField(blank=True, null=True)
    connection = models.TextField(blank=True, null=True)
    device_type = models.TextField(blank=True, null=True)
    gcode_file = models.TextField(blank=True, null=True)
    percentage = models.TextField(blank=True, null=True)
    bed_temperature = models.FloatField(blank=True, null=True)
    time_total = models.TextField(blank=True, null=True)

    class Meta:
        managed = False
        db_table = 'Printer'
