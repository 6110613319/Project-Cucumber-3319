# Project-3319
#### นางสาว ณัฐพร วิลอนุพงษ์ 6110613319
### Motivation
> เนื่องจากสถานการณ์โควิด 19 ทำให้ต้องหันมาเรียน หรือ ทำงานออนไลน์ ซึ่งนั่งอยู่นิ่งๆเป็นเวลานาน จึงส่งผลต่อสุขภาพ โดยมีการการปวดหลัง เนื่องจากนั่งเป็นเวลานานเกินไปติดต่อกันเป็นเวลานาน
> ทางผู้พัฒนาจึงเล็งเห็นประโยชน์ของการนำเทคโนโลยีมาใช้ โดยให้มีการแจ้งเตือนผ่านทาง LINE Application หากมีการขยับร่างกายในท่าต่างๆ รวมไปถึงมีการเก็บเวลาทั้งหมดที่ได้ขยับตัวใน 1 วัน

### Requirement
1. ต้องการให้มีการทำนายสถานะของร่างกายขณะนั้นว่า นั่ง นอน หรือเดินแล้วทำการส่งผ่านทาง LINE Application 
2. ต้องการให้มีการเก็บเวลาทั้งหมดที่ร่างกายเดินในแต่ละวัน
3. ผู้ใช้สามารถสอบถามเวลาที่ใช้เดิน และ สถานะร่างกายของตนเอง ณ ขนาดนั้นได้

### Design
- ใช้ cucumber rs ในการรับค่า x,y,z ของร่างกาย
- ทำการทำนายสถานะของร่างกาย ณ ตอนนั้น
- ส่งสถานะที่เปลี่ยนแปลงไปจากเดิม แจ้งเตือนผ่านทาง LINE Application
- ทำการเก็บระยะเวลาทั้งหมดลงฐานข้อมูล(Firebase) เมื่อสถานะร่างกายเป็น walking
- สามารถกดที่ richMenu ได้เพื่อประวัติการเดินย้อนหลัง
> **สิ่งที่ใช้**
  - HiveMQ
    - topic : cn466/natthaporn/backache/#
    - payload: การเคลื่อนที่(x,y,z) และสถานะการเคลื่อนไหว
  - Edge Impulse
  - LINE Bot& LIFF 
  - Cucumber RS
  - Cloud service 
    - Heroku 
    - Firebase
  - Web API 
    - Fetch API

### Test

