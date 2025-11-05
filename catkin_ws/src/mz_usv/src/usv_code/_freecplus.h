#ifndef __FREECPLUS_H
#define __FREECPLUS_H 1

#include "_cmpublic.h"

///////////////////////////////////// /////////////////////////////////////
// �������ַ���������صĺ�������

// ��ȫ��strcpy������
// dest��Ŀ���ַ���������Ҫ��ʼ������STRCPY�����л�������г�ʼ����
// destlen��Ŀ���ַ���destռ���ڴ�Ĵ�С��
// src��ԭ�ַ�����
// ����ֵ��Ŀ���ַ���dest�ĵ�ַ��
char *STRCPY(char* dest,const size_t destlen,const char* src);

// ��ȫ��strncpy������
// dest��Ŀ���ַ���������Ҫ��ʼ������STRNCPY�����л�������г�ʼ����
// destlen��Ŀ���ַ���destռ���ڴ�Ĵ�С��
// src��ԭ�ַ�����
// n�������Ƶ��ֽ�����
// ����ֵ��Ŀ���ַ���dest�ĵ�ַ��
char *STRNCPY(char* dest,const size_t destlen,const char* src,size_t n);

// ��ȫ��strcat������
// dest��Ŀ���ַ�����
// destlen��Ŀ���ַ���destռ���ڴ�Ĵ�С��
// src����׷�ӵ��ַ�����
// ����ֵ��Ŀ���ַ���dest�ĵ�ַ��
char *STRCAT(char* dest,const size_t destlen,const char* src);

// ��ȫ��strncat������
// dest��Ŀ���ַ�����
// destlen��Ŀ���ַ���destռ���ڴ�Ĵ�С��
// src����׷�ӵ��ַ�����
// n����׷�ӵ��ֽ�����
// ����ֵ��Ŀ���ַ���dest�ĵ�ַ��
char *STRNCAT(char* dest,const size_t destlen,const char* src,size_t n);

// ��ȫ��sprintf������
// ���ɱ����(...)����fmt�����ĸ�ʽ�����dest�ַ����С�
// dest������ַ���������Ҫ��ʼ������SPRINTF�����л�������г�ʼ����
// destlen������ַ���destռ���ڴ�Ĵ�С�������ʽ������ַ������ȴ���destlen-1����������ݽ�������
// fmt����ʽ����������
// ...����䵽��ʽ��������fmt�еĲ�����
// ����ֵ����ʽ������ַ����ĳ��ȣ�����Աһ�㲻���ķ���ֵ��
int SPRINTF(char *dest,const size_t destlen,const char *fmt,...);

// ��ȫ��snprintf������
// ���ɱ����(...)����fmt�����ĸ�ʽ�����dest�ַ����С�
// dest������ַ���������Ҫ��ʼ������SNPRINTF�����л�������г�ʼ����
// destlen������ַ���destռ���ڴ�Ĵ�С�������ʽ������ַ������ȴ���destlen-1����������ݽ�������
// n���Ѹ�ʽ������ַ�����ȡn-1��ŵ�dest�У����n>destlen-1����ȡdestlen-1��
// fmt����ʽ����������
// ...����䵽��ʽ��������fmt�еĲ�����
// ����ֵ����ʽ������ַ����ĳ��ȣ�����Աһ�㲻���ķ���ֵ��
// ע�⣺windows��linuxƽ̨�µ�snprintf�����ĵ���������n���÷����в�ͬ�������ʽ������ַ����ĳ���
// ����10,����������nȡֵ��10����ô����windowsƽ̨��dest�ĳ��Ƚ���10��linuxƽ̨��dest�ĳ���ȴ��9��
int SNPRINTF(char *dest,const size_t destlen,size_t n,const char *fmt,...);

// ɾ���ַ������ָ�����ַ���
// str����������ַ�����
// chr����Ҫɾ�����ַ���
void DeleteLChar(char *str,const char chr);

// ɾ���ַ����ұ�ָ�����ַ���
// str����������ַ�����
// chr����Ҫɾ�����ַ���
void DeleteRChar(char *str,const char chr);

// ɾ���ַ�����������ָ�����ַ���
// str����������ַ�����
// chr����Ҫɾ�����ַ���
void DeleteLRChar(char *str,const char chr);

// ���ַ����е�Сд��ĸת���ɴ�д�����Բ�����ĸ���ַ���
// str����ת�����ַ�����֧��char[]��string�������͡�
void ToUpper(char *str);
void ToUpper(string &str);

// ���ַ����еĴ�д��ĸת����Сд�����Բ�����ĸ���ַ���
// str����ת�����ַ�����֧��char[]��string�������͡�
void ToLower(char *str);
void ToLower(string &str);

// �ַ����滻������
// ���ַ���str�У���������ַ���str1�����滻Ϊ�ַ���str2��
// str����������ַ�����
// str1���ɵ����ݡ�
// str2���µ����ݡ�
// bloop���Ƿ�ѭ��ִ���滻��
// ע�⣺
// 1�����str2��str1Ҫ�����滻��str��䳤�����Ա��뱣֤str���㹻�Ŀռ䣬�����ڴ�������
// 2�����str2�а�����str1�����ݣ���bloopΪtrue���������������߼�����UpdateStr��ʲôҲ������
void UpdateStr(char *str,const char *str1,const char *str2,const bool bloop=true);

// ��һ���ַ�������ȡ�����֡����ź�С���㣬��ŵ���һ���ַ����С�
// src��ԭ�ַ�����
// dest��Ŀ���ַ�����
// bsigned���Ƿ�������ţ�+��-����true-������false-��������
// bdot���Ƿ����С�����Բ����ţ�true-������false-��������
void PickNumber(const char *src,char *dest,const bool bsigned,const bool bdot);

// ������ʽ���ж�һ���ַ����Ƿ�ƥ����һ���ַ�����
// str����Ҫ�жϵ��ַ������Ǿ�ȷ��ʾ�ģ����ļ���"freecplus.cpp"��
// rules��ƥ�����ı��ʽ�����Ǻ�"*"���������ַ�����������ʽ֮���ð�ǵĶ��ŷָ�����"*.h,*.cpp"��
// ע�⣺1��str������֧��"*"��rules����֧��"*"��2���������ж�str�Ƿ�ƥ��rules��ʱ�򣬻������ĸ�Ĵ�Сд��
bool MatchStr(const string str,const string rules);

// ������ʽ���ж�һ���ַ����Ƿ�ƥ����һ���ַ�����
// ����MatchFileName������Ϊ�˼��ݾɵİ汾��
bool MatchFileName(const string in_FileName,const string in_MatchStr);

// ͳ���ַ�����������ȫ�ǵĺ��ֺ�ȫ�ǵı�������һ���֣���ǵĺ��ֺͰ�ǵı�����Ҳ��һ���֡�
// str����ͳ�Ƶ��ַ�����
// ����ֵ���ַ���str��������
int Words(const char *str);
///////////////////////////////////// /////////////////////////////////////


// CCmdStr�����ڲ���зָ������ַ�����
// �ַ����ĸ�ʽΪ���ֶ�����1+�ָ���+�ֶ�����2+�ָ���+�ֶ�����3+�ָ���+...+�ֶ�����n��
// ���磺"messi,10,striker,30,1.72,68.5,Barcelona"�����������˶�Ա÷�������ϣ�����������
// ���º��롢����λ�á����䡢��ߡ����غ�Ч���ľ��ֲ����ֶ�֮���ð�ǵĶ��ŷָ���
class CCmdStr
{
public:
  vector<string> m_vCmdStr;  // ��Ų�ֺ���ֶ����ݡ�

  CCmdStr();  // ���캯����

  // ���ַ�����ֵ�m_vCmdStr�����С�
  // buffer������ֵ��ַ�����
  // sepstr��buffer�в��õķָ�����ע�⣬sepstr�������������Ͳ����ַ������ַ�������","��" "��"|"��"~!~"��
  // bdelspace����ֺ��Ƿ�ɾ���ֶ�����ǰ��Ŀո�true-ɾ����false-��ɾ����ȱʡɾ����
  void SplitToCmd(const string buffer,const char *sepstr,const bool bdelspace=true);

  // ��ȡ��ֺ��ֶεĸ�������m_vCmdStr�����Ĵ�С��
  int CmdCount();

  // ��m_vCmdStr������ȡ�ֶ����ݡ�
  // inum���ֶε�˳��ţ�����������±꣬��0��ʼ��
  // value����������ĵ�ַ�����ڴ���ֶ����ݡ�
  // ����ֵ��true-�ɹ������inum��ȡֵ������m_vCmdStr�����Ĵ�С������ʧ�ܡ�
  bool GetValue(const int inum,char *value,const int ilen=0); // �ַ�����ilenȱʡֵΪ0��
  bool GetValue(const int inum,int  *value); // int������
  bool GetValue(const int inum,unsigned int *value); // unsigned int������
  bool GetValue(const int inum,long *value); // long������
  bool GetValue(const int inum,unsigned long *value); // unsigned long������
  bool GetValue(const int inum,double *value); // ˫����double��
  bool GetValue(const int inum,bool *value); // bool�͡�

  ~CCmdStr(); // ����������
};
///////////////////////////////////// /////////////////////////////////////


///////////////////////////////////// /////////////////////////////////////
// ����xml��ʽ�ַ����ĺ����塣
// xml��ʽ���ַ������������£�
// <filename>/tmp/freecplus.h</filename><mtime>2020-01-01 12:20:35</mtime><size>18348</size>
// <filename>/tmp/freecplus.cpp</filename><mtime>2020-01-01 10:10:15</mtime><size>50945</size>
// xmlbuffer����������xml��ʽ�ַ�����
// fieldname���ֶεı�ǩ����
// value����������ĵ�ַ�����ڴ���ֶ����ݣ�֧��bool��int��insigned int��long��unsigned long��double��char[]��
// ע�⣬��value��������������Ϊchar []ʱ�����뱣֤value������ڴ��㹻��������ܷ����ڴ���������⣬Ҳ������ilen�����޶���ȡ�ֶ����ݵĳ��ȣ�ilen��ȱʡֵΪ0����ʾ���޳��ȡ�
// ����ֵ��true-�ɹ������fieldname����ָ���ı���������ڣ�����ʧ�ܡ�
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,bool *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,int  *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,unsigned int *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,long *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,unsigned long *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,double *value);
bool GetXMLBuffer(const char *xmlbuffer,const char *fieldname,char *value,const int ilen=0);
///////////////////////////////////// /////////////////////////////////////

///////////////////////////////////// /////////////////////////////////////
/*
  ȡ����ϵͳ��ʱ�䡣
  stime�����ڴ�Ż�ȡ����ʱ���ַ�����
  timetvl��ʱ���ƫ��������λ���룬0��ȱʡֵ����ʾ��ǰʱ�䣬30��ʾ��ǰʱ��30��֮���ʱ��㣬-30��ʾ��ǰʱ��30��֮ǰ��ʱ��㡣
  fmt�����ʱ��ĸ�ʽ��fmtÿ���ֵĺ��壺yyyy-��ݣ�mm-�·ݣ�dd-���ڣ�hh24-Сʱ��mi-���ӣ�ss-�룬
  ȱʡ��"yyyy-mm-dd hh24:mi:ss"��Ŀǰ֧�����¸�ʽ��
  "yyyy-mm-dd hh24:mi:ss"
  "yyyymmddhh24miss"
  "yyyy-mm-dd"
  "yyyymmdd"
  "hh24:mi:ss"
  "hh24miss"
  "hh24:mi"
  "hh24mi"
  "hh24"
  "mi"
  ע�⣺
    1��Сʱ�ı�ʾ������hh24������hh����ô����Ŀ����Ϊ�˱��������ݿ��ʱ���ʾ����һ�£�
    2�������г��˳��õ�ʱ���ʽ���������������Ӧ�ÿ������������޸�Դ����timetostr�������Ӹ���ĸ�ʽ֧�֣�
    3�����ú�����ʱ�����fmt��������ʽ��ƥ�䣬stime�����ݽ�Ϊ�ա�
    4��ʱ����������λ�������Ŀ�����һλ����λ�����������λ����ǰ�油0��
*/
void LocalTime(char *stime,const char *fmt=0,const int timetvl=0);

// ��������ʾ��ʱ��ת��Ϊ�ַ�����ʾ��ʱ�䡣
// ltime��������ʾ��ʱ�䡣
// stime���ַ�����ʾ��ʱ�䡣
// fmt������ַ���ʱ��stime�ĸ�ʽ����LocalTime������fmt������ͬ�����fmt�ĸ�ʽ����ȷ��stime��Ϊ�ա�
void timetostr(const time_t ltime,char *stime,const char *fmt=0);

// ���ַ�����ʾ��ʱ��ת��Ϊ������ʾ��ʱ�䡣
// stime���ַ�����ʾ��ʱ�䣬��ʽ���ޣ���һ��Ҫ����yyyymmddhh24miss��һ���������٣�˳��Ҳ���ܱ䡣
// ����ֵ��������ʾ��ʱ�䣬���stime�ĸ�ʽ����ȷ������-1��
time_t strtotime(const char *stime);

// ���ַ�����ʾ��ʱ�����һ��ƫ�Ƶ�������õ�һ���µ��ַ�����ʾ��ʱ�䡣
// in_stime��������ַ�����ʽ��ʱ�䣬��ʽ���ޣ���һ��Ҫ����yyyymmddhh24miss��һ���������٣�˳��Ҳ���ܱ䡣
// out_stime��������ַ�����ʽ��ʱ�䡣
// timetvl����Ҫƫ�Ƶ���������������ƫ�ƣ�������ǰƫ�ơ�
// fmt������ַ���ʱ��out_stime�ĸ�ʽ����LocalTime������fmt������ͬ��
// ע�⣺in_stime��out_stime����������ͬһ�������ĵ�ַ���������ʧ�ܣ�out_stime�����ݻ���ա�
// ����ֵ��true-�ɹ���false-ʧ�ܣ��������ʧ�ܣ�������Ϊ��in_stime�ĸ�ʽ����ȷ��
bool AddTime(const char *in_stime,char *out_stime,const int timetvl,const char *fmt=0);
///////////////////////////////////// /////////////////////////////////////

///////////////////////////////////// /////////////////////////////////////
// ����һ����ȷ��΢��ļ�ʱ����
class CTimer
{
private:
  struct timeval m_start;   // ��ʼ��ʱ��ʱ�䡣
  struct timeval m_end;     // ��ʱ��ɵ�ʱ�䡣
  double m_dstart,m_dend;   // double�͵Ŀ�ʱ�����ʱ�䡣

  // ��ʼ��ʱ��
  void Start();
public:
  CTimer();  // ���캯���л����Start������

  // ��������ȥ��ʱ�䣬��λ���룬С���������΢�롣
  double Elapsed();
};
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// /////////////////////////////////////
// Ŀ¼������ص���

// ���ݾ���·�����ļ�����Ŀ¼���𼶵Ĵ���Ŀ¼��
// pathorfilename������·�����ļ�����Ŀ¼����
// bisfilename��˵��pathorfilename�����ͣ�true-pathorfilename���ļ�����������Ŀ¼����ȱʡֵΪtrue��
// ����ֵ��true-�ɹ���false-ʧ�ܣ��������ʧ�ܣ�ԭ���д�������������1��Ȩ�޲��㣻2��pathorfilename�������ǺϷ����ļ�����Ŀ¼����3�����̿ռ䲻�㡣
bool MKDIR(const char *pathorfilename,bool bisfilename=true);


// ��ȡĳĿ¼������Ŀ¼�е��ļ��б���Ϣ��
class CDir
{
public:
  char m_DirName[301];        // Ŀ¼�������磺/tmp/root��
  char m_FileName[301];       // �ļ�����������Ŀ¼�������磺data.xml��
  char m_FullFileName[301];   // �ļ�ȫ��������Ŀ¼�������磺/tmp/root/data.xml��
  int  m_FileSize;            // �ļ��Ĵ�С����λ���ֽڡ�
  char m_ModifyTime[21];      // �ļ����һ�α��޸ĵ�ʱ�䣬��stat�ṹ���st_mtime��Ա��
  char m_CreateTime[21];      // �ļ����ɵ�ʱ�䣬��stat�ṹ���st_ctime��Ա��
  char m_AccessTime[21];      // �ļ����һ�α����ʵ�ʱ�䣬��stat�ṹ���st_atime��Ա��
  char m_DateFMT[25];         // �ļ�ʱ����ʾ��ʽ����SetDateFMT�������á�

  vector<string> m_vFileName; // ���OpenDir������ȡ���ľ���·���ļ����嵥��
  int m_pos;                  // �Ѷ�ȡm_vFileName������λ�ã�ÿ����һ��ReadDir����m_pos��1��

  CDir();  // ���캯����

  void initdata(); // ��ʼ����Ա������

  // �����ļ�ʱ��ĸ�ʽ��֧��"yyyy-mm-dd hh24:mi:ss"��"yyyymmddhh24miss"���֣�ȱʡ��ǰ�ߡ�
  void SetDateFMT(const char *in_DateFMT);

  // ��Ŀ¼����ȡĿ¼�е��ļ��б���Ϣ�������m_vFileName�����С�
  // in_DirName�����򿪵�Ŀ¼�������þ���·������/tmp/root��
  // in_MatchStr������ȡ�ļ�����ƥ����򣬲�ƥ����ļ������ԣ�������μ�freecplus��ܵ�MatchStr������
  // in_MaxCount����ȡ�ļ������������ȱʡֵΪ10000����
  // bAndChild���Ƿ�򿪸�����Ŀ¼��ȱʡֵΪfalse-������Ŀ¼��
  // bSort���Ƿ�Ի�ȡ�����ļ��б���m_vFileName�����е����ݣ���������ȱʡֵΪfalse-������
  // ����ֵ��true-�ɹ���false-ʧ�ܣ����in_DirName����ָ����Ŀ¼�����ڣ�OpenDir�����ᴴ����Ŀ¼���������ʧ�ܣ�����false�������ǰ�û���in_DirNameĿ¼�µ���Ŀ¼û�ж�ȡȨ��Ҳ�᷵��false��
  bool OpenDir(const char *in_DirName,const char *in_MatchStr,const unsigned int in_MaxCount=10000,const bool bAndChild=false,bool bSort=false);

  // ����һ���ݹ麯������OpenDir()�ĵ��ã���CDir����ⲿ����Ҫ��������
  bool _OpenDir(const char *in_DirName,const char *in_MatchStr,const unsigned int in_MaxCount,const bool bAndChild);

  // ��m_vFileName�����л�ȡһ����¼���ļ�������ͬʱ��ȡ���ļ��Ĵ�С���޸�ʱ�����Ϣ��
  // ����OpenDir����ʱ��m_vFileName��������գ�m_pos���㣬ÿ����һ��ReadDir����m_pos��1��
  // ��m_posС��m_vFileName.size()������true�����򷵻�false��
  bool ReadDir();

  ~CDir();  // ����������
};

///////////////////////////////////// /////////////////////////////////////

///////////////////////////////////// /////////////////////////////////////
// �ļ�������صĺ�������

// ɾ���ļ�������Linuxϵͳ��rm���
// filename����ɾ�����ļ�����������þ���·�����ļ���������/tmp/root/data.xml��
// times��ִ��ɾ���ļ��Ĵ�����ȱʡ��1�����鲻Ҫ����3����ʵ��Ӧ�õľ��鿴�������ɾ���ļ���1�β��ɹ����ٳ���2���ǿ��Եģ�����ξ����岻���ˡ����У����ִ��ɾ��ʧ�ܣ�usleep(100000)�������ԡ�
// ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�ܵ���Ҫԭ����Ȩ�޲��㡣
// ��Ӧ�ÿ����У�������REMOVE��������remove�⺯����
bool REMOVE(const char *filename,const int times=1);

// �������ļ�������Linuxϵͳ��mv���
// srcfilename��ԭ�ļ�����������þ���·�����ļ�����
// destfilename��Ŀ���ļ�����������þ���·�����ļ�����
// times��ִ���������ļ��Ĵ�����ȱʡ��1�����鲻Ҫ����3����ʵ��Ӧ�õľ��鿴��������������ļ���1�β��ɹ����ٳ���2���ǿ��Եģ�����ξ����岻���ˡ����У����ִ��������ʧ�ܣ�usleep(100000)�������ԡ�
// ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�ܵ���Ҫԭ����Ȩ�޲������̿ռ䲻�������ԭ�ļ���Ŀ���ļ�����ͬһ�����̷�����������Ҳ����ʧ�ܡ�
// ע�⣬���������ļ�֮ǰ�����Զ�����destfilename�����а�����Ŀ¼��
// ��Ӧ�ÿ����У�������RENAME��������rename�⺯����
bool RENAME(const char *srcfilename,const char *dstfilename,const int times=1);

// �����ļ�������Linuxϵͳ��cp���
// srcfilename��ԭ�ļ�����������þ���·�����ļ�����
// destfilename��Ŀ���ļ�����������þ���·�����ļ�����
// ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�ܵ���Ҫԭ����Ȩ�޲������̿ռ䲻����
// ע�⣺
// 1���ڸ������ļ�֮ǰ�����Զ�����destfilename�����е�Ŀ¼����
// 2�������ļ��Ĺ����У�������ʱ�ļ������ķ�����������ɺ��ٸ���Ϊdestfilename�������м�״̬���ļ�����ȡ��
// 3�����ƺ���ļ���ʱ����ԭ�ļ���ͬ����һ����Linuxϵͳcp���ͬ��
bool COPY(const char *srcfilename,const char *dstfilename);

// ��ȡ�ļ��Ĵ�С��
// filename������ȡ���ļ�����������þ���·�����ļ�����
// ����ֵ������ļ������ڻ�û�з���Ȩ�ޣ�����-1���ɹ������ļ��Ĵ�С����λ���ֽڡ�
int FileSize(const char *filename);

// ��ȡ�ļ���ʱ�䡣
// filename������ȡ���ļ�����������þ���·�����ļ�����
// mtime�����ڴ���ļ���ʱ�䣬��stat�ṹ���st_mtime��
// fmt������ʱ��������ʽ����LocalTime������ͬ����ȱʡ��"yyyymmddhh24miss"��
// ����ֵ������ļ������ڻ�û�з���Ȩ�ޣ�����false���ɹ�����true��
bool FileMTime(const char *filename,char *mtime,const char *fmt=0);

// �����ļ����޸�ʱ�����ԡ�
// filename�������õ��ļ�����������þ���·�����ļ�����
// stime���ַ�����ʾ��ʱ�䣬��ʽ���ޣ���һ��Ҫ����yyyymmddhh24miss��һ���������٣�˳��Ҳ���ܱ䡣
// ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�ܵ�ԭ�򱣴���errno�С�
bool UTime(const char *filename,const char *mtime);

// ���ļ���
// FOPEN��������fopen�⺯�����ļ�������ļ����а�����Ŀ¼�����ڣ��ʹ���Ŀ¼��
// FOPEN�����Ĳ����ͷ���ֵ��fopen������ȫ��ͬ��
// ��Ӧ�ÿ����У���FOPEN��������fopen�⺯����
FILE *FOPEN(const char *filename,const char *mode);

// ���ı��ļ��ж�ȡһ�С�
// fp���Ѵ򿪵��ļ�ָ�롣
// buffer�����ڴ�Ŷ�ȡ�����ݣ�buffer�������readsize+1��������ܻ���ɶ��������ݲ��������ڴ�������
// readsize�����δ����ȡ���ֽ���������Ѿ���ȡ�����н�����־���������ء�
// endbz�������ݽ����ı�־��ȱʡΪ�գ���ʾ��������"\n"Ϊ������־��
// ����ֵ��true-�ɹ���false-ʧ�ܣ�һ������£�ʧ�ܿ�����Ϊ���ļ��ѽ�����
bool FGETS(const FILE *fp,char *buffer,const int readsize,const char *endbz=0);

// �ļ�����������
class CFile
{
private:
  FILE *m_fp;        // �ļ�ָ��
  bool  m_bEnBuffer; // �Ƿ����û��壬true-���ã�false-�����ã�ȱʡ�����á�
  char  m_filename[301]; // �ļ�����������þ���·�����ļ�����
  char  m_filenametmp[301]; // ��ʱ�ļ�������m_filename���".tmp"��

public:
  CFile();   // ���캯����

  bool IsOpened();  // �ж��ļ��Ƿ��Ѵ򿪣�����ֵ��true-�Ѵ򿪣�false-δ�򿪡�

  // ���ļ���
  // filename�����򿪵��ļ�����������þ���·�����ļ�����
  // openmode�����ļ���ģʽ����fopen�⺯���Ĵ�ģʽ��ͬ��
  // bEnBuffer���Ƿ����û��壬true-���ã�false-�����ã�ȱʡ�����á�
  // ע�⣺������򿪵��ļ���Ŀ¼�����ڣ��ͻᴴ��Ŀ¼��
  bool Open(const char *filename,const char *openmode,bool bEnBuffer=true);

  // �ر��ļ�ָ�룬��ɾ���ļ���
  bool CloseAndRemove();

  // רΪ�����������ļ���������Open������ͬ��
  // ע�⣺OpenForRename�򿪵���filename���".tmp"����ʱ�ļ�������openmodeֻ����"a"��"a+"��"w"��"w+"��
  bool OpenForRename(const char *filename,const char *openmode,bool bEnBuffer=true);
  // �ر��ļ�ָ�룬����OpenForRename�����򿪵���ʱ�ļ���������Ϊfilename��
  bool CloseAndRename();

  // ����fprintf���ļ�д�����ݣ�������fprintf�⺯����ͬ��������Ҫ�����ļ�ָ�롣
  void Fprintf(const char *fmt,...);

  // ���ļ��ж�ȡ�Ի��з�"\n"������һ�С�
  // buffer�����ڴ�Ŷ�ȡ�����ݣ�buffer�������readsize+1��������ܻ�����ڴ�������
  // readsize�����δ����ȡ���ֽ���������Ѿ���ȡ���˽�����־"\n"���������ء�
  // bdelcrt���Ƿ�ɾ���н�����־"\r"��"\n"��true-ɾ����false-��ɾ����ȱʡֵ��false��
  // ����ֵ��true-�ɹ���false-ʧ�ܣ�һ������£�ʧ�ܿ�����Ϊ���ļ��ѽ�����
  bool Fgets(char *buffer,const int readsize,bool bdelcrt=false);

  // ���ļ��ļ��ж�ȡһ�С�
  // buffer�����ڴ�Ŷ�ȡ�����ݣ�buffer�������readsize+1��������ܻ���ɶ��������ݲ��������ڴ�������
  // readsize�����δ����ȡ���ֽ���������Ѿ���ȡ���˽�����־���������ء�
  // endbz�������ݽ����ı�־��ȱʡΪ�գ���ʾ��������"\n"Ϊ������־��
  // ����ֵ��true-�ɹ���false-ʧ�ܣ�һ������£�ʧ�ܿ�����Ϊ���ļ��ѽ�����
  bool FFGETS(char *buffer,const int readsize,const char *endbz=0);

  // ���ļ��ж�ȡ���ݿ顣
  // ptr�����ڴ�Ŷ�ȡ�����ݡ�
  // size�����δ����ȡ���ֽ�����
  // ����ֵ�����δ��ļ��гɹ���ȡ���ֽ���������ļ�δ����������ֵ����size������ļ��ѽ���������ֵΪʵ�ʶ�ȡ���ֽ�����
  size_t Fread(void *ptr,size_t size);

  // ���ļ���д�����ݿ顣
  // ptr����д�����ݿ�ĵ�ַ��
  // size����д�����ݿ���ֽ�����
  // ����ֵ�����γɹ�д����ֽ�����������̿ռ��㹻������ֵ����size��
  size_t Fwrite(const void *ptr,size_t size);

  // �ر��ļ�ָ�룬���������ʱ�ļ�����ɾ������
  void Close();

 ~CFile();   // �������������Close������
};

///////////////////////////////////// /////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// ��������־�ļ�������

// ��־�ļ�������
class CLogFile
{
public:
  FILE   *m_tracefp;           // ��־�ļ�ָ�롣
  char    m_filename[301];     // ��־�ļ�����������þ���·����
  char    m_openmode[11];      // ��־�ļ��Ĵ򿪷�ʽ��һ�����"a+"��
  bool    m_bEnBuffer;         // д����־ʱ���Ƿ����ò���ϵͳ�Ļ�����ƣ�ȱʡ�����á�
  long    m_MaxLogSize;        // �����־�ļ��Ĵ�С����λM��ȱʡ100M��
  bool    m_bBackup;           // �Ƿ��Զ��л�����־�ļ���С����m_MaxLogSize���Զ��л���ȱʡ���á�

  // ���캯����
  // MaxLogSize�������־�ļ��Ĵ�С����λM��ȱʡ100M����СΪ10M��
  CLogFile(const long MaxLogSize=100);

  // ����־�ļ���
  // filename����־�ļ�����������þ���·��������ļ����е�Ŀ¼�����ڣ����ȴ���Ŀ¼��
  // openmode����־�ļ��Ĵ򿪷�ʽ����fopen�⺯�����ļ��ķ�ʽ��ͬ��ȱʡֵ��"a+"��
  // bBackup���Ƿ��Զ��л���true-�л���false-���л����ڶ���̵ķ�������У����������̹���һ����־�ļ���bBackup����Ϊfalse��
  // bEnBuffer���Ƿ������ļ�������ƣ�true-���ã�false-�����ã�������û���������ôд����־�ļ��е����ݲ�������д���ļ���ȱʡ�ǲ����á�
  bool Open(const char *filename,const char *openmode=0,bool bBackup=true,bool bEnBuffer=false);

  // �����־�ļ�����m_MaxLogSize��ֵ���Ͱѵ�ǰ����־�ļ�����Ϊ��ʷ��־�ļ������ٴ����µĵ�ǰ��־�ļ���
  // ���ݺ���ļ�������־�ļ������������ʱ�䣬��/tmp/log/filetodb.log.20200101123025��
  // ע�⣬�ڶ���̵ĳ����У���־�ļ������л������ߵĳ����У���־�ļ������л���
  bool BackupLogFile();

  // ������д����־�ļ���fmt�ǿɱ������ʹ�÷�����printf�⺯����ͬ��
  // Write������д�뵱ǰ��ʱ�䣬WriteEx������дʱ�䡣
  bool Write(const char *fmt,...);
  bool WriteEx(const char *fmt,...);

  // �ر���־�ļ�
  void Close();

  ~CLogFile();  // �������������Close������
};
///////////////////////////////////////////////////////////////////////////////////////////////////

// �����ļ������ࡣ
// CIniFile������Ĳ��Ǵ�ͳ��ini�ļ�����xml��ʽ�Ĳ����ļ������磺
/*
 * �ļ�����hssms.xml
<?xml version="1.0" encoding="gbk" ?>
<root>
    <!-- �������е���־�ļ����� -->
    <logpath>/log/hssms</logpath>

    <!-- ���ݿ����Ӳ����� -->
    <connstr>hssms/smspwd@hssmszx</connstr>

    <!-- �����ļ���ŵĸ�Ŀ¼�� -->
    <datapath>/data/hssms</datapath>

    <!-- ���ķ�������ip��ַ�� -->
    <serverip>192.168.1.1</serverip>

    <!-- ���ķ�������ͨ�Ŷ˿ڡ� -->
    <port>5058</port>

    <!-- �Ƿ���ó����ӣ�true-�ǣ�false-�� -->
    <online>true</online>
</root>
*/

class CIniFile
{
public:
  string m_xmlbuffer; // ��Ų����ļ�ȫ�������ݣ���LoadFile�������롣

  CIniFile();

  // �Ѳ����ļ����������뵽m_xmlbuffer��Ա�����С�
  bool LoadFile(const char *filename);

  // ��ȡ������ֵ��
  // fieldname���ֶεı�ǩ����
  // value����������ĵ�ַ�����ڴ���ֶε�ֵ��֧��bool��int��insigned int��long��unsigned long��double��char[]��
  // ע�⣬��value��������������Ϊchar []ʱ�����뱣֤value���ڴ��㹻��������ܷ����ڴ���������⣬
  // Ҳ������ilen�����޶���ȡ�ֶ����ݵĳ��ȣ�ilen��ȱʡֵΪ0����ʾ���޶���ȡ�ֶ����ݵĳ��ȡ�
  // ����ֵ��true-�ɹ���false-ʧ�ܡ�
  bool GetValue(const char *fieldname,bool *value);
  bool GetValue(const char *fieldname,int  *value);
  bool GetValue(const char *fieldname,unsigned int *value);
  bool GetValue(const char *fieldname,long *value);
  bool GetValue(const char *fieldname,unsigned long *value);
  bool GetValue(const char *fieldname,double *value);
  bool GetValue(const char *fieldname,char *value,const int ilen=0);
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// ������socketͨ�ŵĺ�������

// socketͨ�ŵĿͻ�����
class CTcpClient
{
public:
  int  m_sockfd;    // �ͻ��˵�socket.
  char m_ip[21];    // ����˵�ip��ַ��
  int  m_port;      // ������ͨ�ŵĶ˿ڡ�
  bool m_btimeout;  // ����Read��Write����ʱ��ʧ�ܵ�ԭ���Ƿ��ǳ�ʱ��true-��ʱ��false-δ��ʱ��
  int  m_buflen;    // ����Read�����󣬽��յ��ı��ĵĴ�С����λ���ֽڡ�

  CTcpClient();  // ���캯����

  // �����˷�����������
  // ip������˵�ip��ַ��
  // port������˼����Ķ˿ڡ�
  // ����ֵ��true-�ɹ���false-ʧ�ܡ�
  bool ConnectToServer(const char *ip,const int port);

  // ���շ���˷��͹��������ݡ�
  // buffer���������ݻ������ĵ�ַ�����ݵĳ��ȴ����m_buflen��Ա�����С�
  // itimeout���ȴ����ݵĳ�ʱʱ�䣬��λ���룬ȱʡֵ��0-���޵ȴ���
  // ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�������������1���ȴ���ʱ����Ա����m_btimeout��ֵ������Ϊtrue��2��socket�����Ѳ����á�
  bool Read(char *buffer,const int itimeout=0);

  // �����˷������ݡ�
  // buffer�����������ݻ������ĵ�ַ��
  // ibuflen�����������ݵĴ�С����λ���ֽڣ�ȱʡֵΪ0��������͵���ascii�ַ�����ibuflenȡ0������Ƕ����������ݣ�ibuflenΪ���������ݿ�Ĵ�С��
  // ����ֵ��true-�ɹ���false-ʧ�ܣ����ʧ�ܣ���ʾsocket�����Ѳ����á�
  bool Write(const char *buffer,const int ibuflen=0);

  // �Ͽ������˵�����
  void Close();

  ~CTcpClient();  // ���������Զ��ر�socket���ͷ���Դ��
};

// socketͨ�ŵķ������
class CTcpServer
{
private:
  int m_socklen;                    // �ṹ��struct sockaddr_in�Ĵ�С��
  struct sockaddr_in m_clientaddr;  // �ͻ��˵ĵ�ַ��Ϣ��
  struct sockaddr_in m_servaddr;    // ����˵ĵ�ַ��Ϣ��
public:
  int  m_listenfd;   // ��������ڼ�����socket��
  int  m_connfd;     // �ͻ�������������socket��
  bool m_btimeout;   // ����Read��Write����ʱ��ʧ�ܵ�ԭ���Ƿ��ǳ�ʱ��true-��ʱ��false-δ��ʱ��
  int  m_buflen;     // ����Read�����󣬽��յ��ı��ĵĴ�С����λ���ֽڡ�

  CTcpServer();  // ���캯����

  // ����˳�ʼ����
  // port��ָ����������ڼ����Ķ˿ڡ�
  // ����ֵ��true-�ɹ���false-ʧ�ܣ�һ������£�ֻҪport������ȷ��û�б�ռ�ã���ʼ������ɹ���
  bool InitServer(const unsigned int port);

  // �����ȴ��ͻ��˵���������
  // ����ֵ��true-���µĿͻ���������������false-ʧ�ܣ�Accept���жϣ����Acceptʧ�ܣ���������Accept��
  bool Accept();

  // ��ȡ�ͻ��˵�ip��ַ��
  // ����ֵ���ͻ��˵�ip��ַ����"192.168.1.100"��
  char *GetIP();

  // ���տͻ��˷��͹��������ݡ�
  // buffer���������ݻ������ĵ�ַ�����ݵĳ��ȴ����m_buflen��Ա�����С�
  // itimeout���ȴ����ݵĳ�ʱʱ�䣬��λ���룬ȱʡֵ��0-���޵ȴ���
  // ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�������������1���ȴ���ʱ����Ա����m_btimeout��ֵ������Ϊtrue��2��socket�����Ѳ����á�
  bool Read(char *buffer,const int itimeout=0);

  // ��ͻ��˷������ݡ�
  // buffer�����������ݻ������ĵ�ַ��
  // ibuflen�����������ݵĴ�С����λ���ֽڣ�ȱʡֵΪ0��������͵���ascii�ַ�����ibuflenȡ0������Ƕ����������ݣ�ibuflenΪ���������ݿ�Ĵ�С��
  // ����ֵ��true-�ɹ���false-ʧ�ܣ����ʧ�ܣ���ʾsocket�����Ѳ����á�
  bool Write(const char *buffer,const int ibuflen=0);

  // �رռ�����socket����m_listenfd�������ڶ���̷��������ӽ��̴����С�
  void CloseListen();

  // �رտͻ��˵�socket����m_connfd�������ڶ���̷������ĸ����̴����С�
  void CloseClient();

  ~CTcpServer();  // ���������Զ��ر�socket���ͷ���Դ��
};

// ����socket�ĶԶ˷��͹��������ݡ�
// sockfd�����õ�socket���ӡ�
// buffer���������ݻ������ĵ�ַ��
// ibuflen�����γɹ��������ݵ��ֽ�����
// itimeout�����յȴ���ʱ��ʱ�䣬��λ���룬ȱʡֵ��0-���޵ȴ���
// ����ֵ��true-�ɹ���false-ʧ�ܣ�ʧ�������������1���ȴ���ʱ��2��socket�����Ѳ����á�
bool TcpRead(const int sockfd,char *buffer,int *ibuflen,const int itimeout=0);

// ��socket�ĶԶ˷������ݡ�
// sockfd�����õ�socket���ӡ�
// buffer�����������ݻ������ĵ�ַ��
// ibuflen�����������ݵ��ֽ�����������͵���ascii�ַ�����ibuflenȡ0������Ƕ����������ݣ�ibuflenΪ���������ݿ�Ĵ�С��
// ����ֵ��true-�ɹ���false-ʧ�ܣ����ʧ�ܣ���ʾsocket�����Ѳ����á�
bool TcpWrite(const int sockfd,const char *buffer,const int ibuflen=0);

// ���Ѿ�׼���õ�socket�ж�ȡ���ݡ�
// sockfd���Ѿ�׼���õ�socket���ӡ�
// buffer���������ݻ������ĵ�ַ��
// n�����ν������ݵ��ֽ�����
// ����ֵ���ɹ����յ�n�ֽڵ����ݺ󷵻�true��socket���Ӳ����÷���false��
bool Readn(const int sockfd,char *buffer,const size_t n);

// ���Ѿ�׼���õ�socket��д�����ݡ�
// sockfd���Ѿ�׼���õ�socket���ӡ�
// buffer�����������ݻ������ĵ�ַ��
// n�����������ݵ��ֽ�����
// ����ֵ���ɹ�������n�ֽڵ����ݺ󷵻�true��socket���Ӳ����÷���false��
bool Writen(const int sockfd,const char *buffer,const size_t n);

// ������socketͨ�ŵĺ�������
///////////////////////////////////// /////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// ���´��������ļ�����ϵͳ��������freecplus��ܡ�

// �ر�ȫ�����źź��������
void CloseIOAndSignal();

// �ļ���Ϣ�����ݽṹ
// ȫ·���ļ�������С��ʱ��Ľṹ��
struct st_fileinfo
{
  char filename[301];
  int  filesize;
  char mtime[21];
};

// ��socket�ĶԶ˷����ļ���
// sockfd�����õ�socket���ӡ�
// stfileinfo�������͵��ļ���Ϣ����struct st_fileinfo��ʾ��
// logfile�����ڼ�¼�������־�ļ���ָ�룬���Ϊ0�������˴��󲻼�¼��־��
// ����ֵ��true-�ɹ���false-����ʧ�ܣ�ʧ�ܵ�ԭ�������֣�1��sockfd�����ã�2�������͵��ļ������ڻ�Ȩ�޲��㡣
bool SendFile(int sockfd,struct st_fileinfo *stfileinfo,CLogFile *logfile=0);

// ����socket�ĶԶ˷��͹������ļ���
// sockfd�����õ�socket���ӡ�
// stfileinfo�������յ��ļ���Ϣ����struct st_fileinfo��ʾ��
// logfile�����ڼ�¼�������־�ļ���ָ�룬���Ϊ0�������˴��󲻼�¼��־��
// ����ֵ��true-�ɹ���false-����ʧ�ܣ�ʧ�ܵ�ԭ�������֣�1��sockfd�����ã�2�������͵��ļ������ڻ�Ȩ�޲��㡣
bool RecvFile(int sockfd,struct st_fileinfo *stfileinfo,CLogFile *logfile=0);
///////////////////////////////////// /////////////////////////////////////


#endif
