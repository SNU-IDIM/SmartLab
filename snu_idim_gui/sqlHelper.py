# Python 3
# -*- coding: utf-8 -*-

import pymysql
import os
import csv

class SqlHelper():
    def __init__(self, host="localhost", username="root", password="", port=3306, database=None, charset="utf8mb4"):
        self.host = host
        self.username = username
        self.password = password
        self.database = database
        self.port = port
        self.con = None
        self.cur = None

        try:
            self.con = pymysql.connect(host=self.host, user=self.username, passwd=self.password, port=self.port, charset=charset)
            self.cur = self.con.cursor()
            if database is not None:
                self.select_db(database)
        except:
            raise "[ERROR] DataBase connection error !!!"


    def select_db(self, database):
        self.cur.execute('CREATE DATABASE IF NOT EXISTS {}'.format(database))
        self.cur.execute('USE {}'.format(database))
        self.database = database
        print("[DEBUG] Selected DB: {}".format(self.database))


    def close(self):
        if not self.con:
            self.con.close()


    def get_version(self):
        self.cur.execute("SELECT VERSION()")
        version = self.get_one_data()[0]
        print("[DEBUG] MySQL Version: {}".format(version))
        return version


    def get_one_data(self):
        data = self.cur.fetchone()
        return data


    def fetch_one_data(self, tablename):
        sql = "SELECT * FROM {}".format(tablename)
        print('[SQL] {}'.format(sql))
        self.execute_commit(sql)
        return self.cur.fetchone()

    def create_table(self, tablename, attrdict, constraint):
        """创建数据库表
            args：
                tablename  ：表名字
                attrdict   ：属性键值对,{'book_name':'varchar(200) NOT NULL'...}
                constraint ：主外键约束,PRIMARY KEY(`id`)
        """
        if self.is_exist_table(tablename):
            return
        sql = ''
        sql_mid = ''
        for attr,value in attrdict.items():
            # sql_mid += '`'+attr + '`'+' '+ value+','
            sql_mid += "{} {}, ".format(attr, value)#  '`'+attr + '`'+' '+ value+','
        sql += 'CREATE TABLE IF NOT EXISTS {} ('.format(tablename)
        sql += sql_mid + constraint 
        sql += ') ENGINE=InnoDB DEFAULT CHARSET=utf8mb4'
        print('[SQL] {}'.format(sql))
        self.execute_commit(sql)

    def recreate_table(self, tablename, attrdict, constraint):
        if not self.is_exist_table(tablename):
            return self.create_table(tablename, attrdict, constraint)

        self.backup_table(tablename, tablename + '.csv')
        self.delete_table(tablename)
        self.create_table(tablename, attrdict, constraint)
        self.restore_table(tablename, tablename + '.csv')
        os.remove(tablename + '.csv')

    def execute_sql(self, sql=''):
        """执行sql语句，针对读操作返回结果集
            args：
                sql  ：sql语句
        """
        try:
            self.cur.execute(sql)
            records = self.cur.fetchall()
            return records
        except pymysql.Error as e:
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            print(error)

    def execute_sql_file(self, sql):
        """ 执行 sql'文件中的sql
        """
        if not os.path.isfile(sql):
            raise 'invalid input file path: %s' % sql
        stmts = []
        stmt = ''
        delimiter = ';'
        commentsblock = False
        with open(sql, 'r', encoding='utf-8') as f:
            data = f.readlines()
            for lineno, line in enumerate(data):
                if not line.strip():
                    continue
                if line.startswith('--'):
                    continue
                if line.startswith('/*'):
                    commentsblock = True
                    continue
                if line.endswith('*/'):
                    commentsblock = False
                    continue
                if 'DELIMITER' in line:
                    delimiter = line.split()[1]
                    continue
                if delimiter not in line:
                    stmt += line.replace(delimiter, ';')
                    continue

                if stmt:
                    stmt += line
                    stmts.append(stmt.strip())
                    stmt = ''
                else:
                    stmts.append(line.strip())

        try:
            for s in stmts:
                self.cur.execute(s)
            self.con.commit()
        except pymysql.Error as e:
            self.con.rollback()
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            raise error

    def execute_commit(self,sql=''):
        """执行数据库sql语句，针对更新,删除,事务等操作失败时回滚
        """
        try:
            self.cur.execute(sql)
            self.con.commit()
        except pymysql.Error as e:
            self.con.rollback()
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            print("error:", error)
            return error

    def insert(self, tablename, params):
        """
            example:
                mydb.select(table, param={"column1": data1})
                mydb.select(table, param={"column1": data1, "column2": data2})
        """
        if params is not None:
            if not isinstance(params, (dict, list, tuple)) or len(params) == 0:
                e = SQLValueError('expression insert', 'invalid values')
                raise e

            attrs_sql = ''
            if isinstance(params, dict):
                columns = [k.strip('\'"') for k in params.keys()]
                print(columns)
                attrs_sql = f"{','.join(columns)}"
                values = [v.decode('utf8') if isinstance(v, bytes) else v for v in params.values()]
            else:
                values = [v.decode('utf8') if isinstance(v, bytes) else v for v in params]
            values_sql = f'{tuple(values)}'
            if values_sql[-2:].find(',') != -1:
                values_sql = f'{tuple(values)}'[:-2] + ')'
                print("@@@@@@@@@@@@@@@@@@")

        sql = 'INSERT INTO {} ({}) VALUES {}'.format(tablename, attrs_sql, values_sql)
        print('[SQL] {}'.format(sql))
        self.execute_commit(sql)

    def select(self, tablename, fields='*', conds='', order=''):
        """
            example：
                print mydb.select(table)
                print mydb.select(table, fields=["name"])
                print mydb.select(table, fields=["name", "age"])
                print mydb.select(table, fields=["age", "name"])
                print mydb.select(table, fields=["age", "name"], conds = ["name = 'usr_name'", "age < 30"])
        """
        sql = ','.join(fields) if isinstance(fields, list) else fields
        sql = 'SELECT {} FROM {} '.format(sql, tablename)

        consql = ''
        if conds != '':
            if isinstance(conds, list):
                conds = ' AND '.join(conds)
                consql = 'WHERE ' + conds
            else:
                consql = 'WHERE ' + conds

        sql += consql + order
        print('[SQL] {}'.format(sql))
        records = self.execute_sql(sql)
        if records is None or len(records) == 0:
            return None
        if len(records[0]) == 1:
            return tuple([r for r, in records])

        columns = self.get_table_columns(tablename) if fields == '*' else fields
        data = tuple([dict(zip(columns, row)) for row in records])
        print('[DEBUG] Selected data: \n{}'.format(data))

        return data

    def insert_many(self, table, attrs = None, values = None, datalist = None):
        """插入多条数据
            example：
                table='test_mysqldb'
                key = ["id" ,"name", "age"]
                value = [[101, "liuqiao", "25"], [102,"liuqiao1", "26"], [103 ,"liuqiao2", "27"], [104 ,"liuqiao3", "28"]]
                mydb.insert_many(table, key, value)
                data = [{"id":101,"name":"liuqiao","age":"25"},{"id":102,"name":"liuqiao1", "age":"26"}]
                nydb.insert_many(table, datalist=data)
        """
        if datalist is not None:
            if not isinstance(datalist, (list, tuple)):
                self.insert(table, datalist)
                return

            if len(datalist) == 1:
                self.insert(table, datalist[0])
                return

            keys = datalist[0].keys()
            re_data = []
            cand_data = []
            for d in datalist:
                if not len(d.keys()) == len(keys):
                    re_data.append(d)
                    continue
                keys_same = True
                for k in keys:
                    if not k in d.keys():
                        re_data.append(d)
                        keys_same = False
                        continue
                if keys_same:
                    cand_data.append(d)

            cand_list = []
            for d in cand_data:
                d_list = []
                for k in keys:
                    d_list.append(d[k])
                cand_list.append(d_list)

            self.insert_many(table, keys, cand_list)

            if len(re_data):
                self.insert_many(table, datalist = re_data)

        if attrs and values:
            values_sql = ['%s' for v in attrs]
            attrs_sql = '('+','.join(attrs)+')'
            values_sql = ' values('+','.join(values_sql)+')'
            sql = 'insert into %s'% table
            sql = sql + attrs_sql + values_sql
            #print('insert_many:'+sql)
            try:
                for i in range(0,len(values),20000):
                    self.cur.executemany(sql,values[i:i+20000])
                    self.con.commit()
            except pymysql.Error as e:
                self.con.rollback()
                error = 'insert_many executemany failed! ERROR (%s): %s' %(e.args[0],e.args[1])
                print(error)

    def delete(self, tablename, cond_dict):
        """删除数据
            args：
                tablename  ：表名字
                cond_dict  ：删除条件字典
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                mydb.delete(table, params)
        """
        consql = ' '
        if cond_dict!='':
            for k, v in cond_dict.items():
                if isinstance(v, str):
                    v = "\'" + v + "\'"
                consql = consql + tablename + "." + k + '=' + v + ' and '
        consql = consql + ' 1=1 '
        sql = "DELETE FROM %s where%s" % (tablename, consql)
        #print (sql)
        return self.execute_commit(sql)

    def update(self, tablename, attrs_dict, cond_dict):
        """更新数据
            args：
                tablename  ：表名字
                attrs_dict  ：更新属性键值对字典
                cond_dict  ：更新条件字典
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                cond_dict = {"name" : "liuqiao", "age" : "18"}
                mydb.update(table, params, cond_dict)
        """
        attrs_list = []
        consql = ' '
        for tmpkey, tmpvalue in attrs_dict.items():
            attrs_list.append("`" + tmpkey + "`" + "=" +"\'" + str(tmpvalue) + "\'")
        attrs_sql = ",".join(attrs_list)
        #print("attrs_sql:", attrs_sql)
        if cond_dict!='':
            for k, v in cond_dict.items():
                v = "\'" + str(v) + "\'"
                consql = consql + "`" + tablename +"`." + "`" + str(k) + "`" + '=' + v + ' and '
        consql = consql + ' 1=1 '
        sql = "UPDATE %s SET %s where%s" % (tablename, attrs_sql, consql)
        #print(sql)
        return self.execute_commit(sql)

    def update_many(self, table, conkeys, attrs = None, values = None, datalist = None):
        """更新多条数据, 有重复则
            args：
                tablename  : 表名字
                attrs      : 属性键
                conkeys    : 条件属性键
                values     : 所有属性值
                datalist   : dict list of data
            example：
                table='test_mysqldb'
                keys = ["name", "age"]
                conkeys = ["id"]
                values = [["liuqiao", "25", 101], ["liuqiao1", "26", 102], ["liuqiao2", "27", 103], ["liuqiao3", "28", 104]]
                mydb.update_many(table, conkeys, keys, values)
        """

        if datalist is not None:
            if not isinstance(datalist, (list, tuple)):
                print('update_many Error: datalist only accept list or tuple, but get ', type(datalist))
                return

            if len(datalist) == 1:
                attrs_dict = {k: v for k, v in datalist[0].items() if k not in conkeys and v is not None}
                cond_dict = {k: v for k, v in datalist[0].items() if k in conkeys and v is not None}
                self.update(table, attrs_dict, cond_dict)
                return

            keys = datalist[0].keys()
            re_data = []
            cand_data = []
            for d in datalist:
                if not len(d.keys()) == len(keys):
                    re_data.append(d)
                    continue
                keys_same = True
                for k in keys:
                    if not k in d.keys():
                        re_data.append(d)
                        keys_same = False
                        continue
                if keys_same:
                    cand_data.append(d)

            cand_list = []
            attr_keys = [k for k in keys if k not in conkeys]
            for d in cand_data:
                d_list = []
                for k in attr_keys:
                    d_list.append(d[k])
                for k in conkeys:
                    d_list.append(d[k])
                cand_list.append(d_list)

            self.update_many(table, conkeys = conkeys, attrs = attr_keys, values = cand_list)

            if len(re_data):
                self.update_many(table, conkeys = conkeys, datalist = re_data)

        if attrs is not None and values is not None:
            attrs_list = [a + '=(%s)' for a in attrs]
            attrs_sql = ','.join(attrs_list)
            cond_list = [c + '=(%s)' for c in conkeys]
            cond_sql = ' and '.join(cond_list)
            sql = "UPDATE %s SET %s where %s" % (table, attrs_sql, cond_sql)
            try:
                for i in range(0,len(values),20000):
                    self.cur.executemany(sql, values[i:i+20000])
                    self.con.commit()
            except pymysql.Error as e:
                self.con.rollback()
                error = 'update_many executemany failed! ERROR (%s): %s' %(e.args[0],e.args[1])
                print(error)

    def drop_table(self, tablename):
        """删除数据库表
            args：
                tablename  ：表名字
        """
        sql = "DROP TABLE  %s" % tablename
        self.execute_commit(sql)

    def delete_table(self, tablename):
        """清空数据库表
            args：
                tablename  ：表名字
        """
        sql = "DELETE FROM %s" % tablename
        self.execute_commit(sql)

    def is_exist_schema(self, database):
        result, = self.select("information_schema.SCHEMATA", "count(*)",["schema_name = '%s'" % database]);
        return result > 0

    def is_exist_table(self, tablename):
        """判断数据表是否存在
            args：
                tablename  ：表名字
            Return:
                存在返回True，不存在返回False
        """
        result, = self.select("information_schema.tables","count(*)",["table_name = '%s'" % tablename, "table_schema = '%s'" % self.database])
        return result > 0

    def is_exist_table_column(self, tablename, column_name):
        result, = self.select("information_schema.columns","count(*)",["table_name = '%s'" % tablename, "column_name = '%s'" % column_name, "table_schema = '%s'" % self.database])
        return result > 0

    def is_exist_table_rows(self, tablename, clauses):
        result = self.select(tablename, '*', conds = clauses)
        return False if result is None or len(result) == 0 else len(result[0]) > 0

    def get_table_columns(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '%s'" % self.database, "table_name = '%s'" % tablename])

    def get_table_columns_prim(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '%s'" % self.database, "table_name = '%s'" % tablename, "column_key = 'PRI'"])

    def add_column(self, tablename, col, tp):
        sql = "alter table %s add %s %s" % (tablename, col, tp)
        self.execute_commit(sql)

    def delete_column(self, tablename, col):
        sql = "alter table %s drop column %s" % (tablename, col)
        self.execute_commit(sql)

    def backup_table(self, tablename, file):
        columns = self.get_table_columns(tablename)
        expdata = tuple([tuple([x for x in columns])])
        data = self.select(tablename, '*')
        expdata += tuple([tuple(d.values()) for d in data])
        with open(file, 'wt', encoding='utf-8') as bkf:
           csv_out = csv.writer(bkf)
           csv_out.writerows(expdata)

    def restore_table(self, tablename, file, existing_rows='update'):
        if not self.is_exist_table(tablename):
            print('table: %s not exists, by pass %s' % (tablename, file))
            return
        if not os.path.isfile(file):
            print('file: %s not exists, can not import to table %s' % (file, tbl))
            return
        
        pri_keys = self.get_table_columns_prim(tablename)
        pri_keys = [k for k in pri_keys]
        col_names = []
        data_insert = []
        data_update = []
        with open(file, 'rt', encoding='utf-8') as bkf:
            csv_reader = csv.reader(bkf)
            pri_idx = []
            line_count = 0
            for row in csv_reader:
                if len(row) == 0:
                    continue
                if line_count == 0:
                    col_names = row
                    pri_idx = [ [k, col_names.index(k)] for k in pri_keys]
                    line_count += 1
                else:
                    clause = ''
                    pri_vals = []
                    for [k, i] in pri_idx:
                        clause += str(k) + '= \'' + str(row[i]) + '\' and '
                        pri_vals.append(row[i])
                    clause += ' 1=1 '
                    if self.is_exist_table_rows(tablename, clauses = clause):
                        if existing_rows == 'update':
                            data_update.append(row)
                    else:
                        data_insert.append(row)
        
        self.execute_commit('SET FOREIGN_KEY_CHECKS=0')
        if len(data_insert) > 0:
            datalist = [{c:v for c,v in zip(col_names, row)} for row in data_insert]
            self.insert_many(tablename, datalist=datalist)

        if len(data_update) > 0:
            datalist = [{c:v for c,v in zip(col_names, row)} for row in data_update]
            self.update_many(tablename, conkeys = pri_keys, datalist = datalist)

        self.execute_commit('SET FOREIGN_KEY_CHECKS=1')



if __name__ == "__main__":
    mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306)
    mysql.get_version()

    mysql.select_db('SmartLab')
    if not mysql.is_exist_table('device_info'):
        
        mysql.create_table(tablename='device_info', attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp', 'time_stamp2': 'timestamp'}, constraint="PRIMARY KEY(id)")

    print(mysql.get_table_columns(tablename='device_info'))
    mysql.insert('device_info', {'time_stamp': '2021-04-25 00:14:10', 'time_stamp2': '2021-04-25 00:14:10'})
    mysql.select('device_info')