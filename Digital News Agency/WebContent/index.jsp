<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
<sql:setDataSource var="ds" driver="com.mysql.jdbc.Driver"
     url="jdbc:mysql://localhost/dna"
     user="root"  password="" scope="application"/>
<%@ include file="footer.jsp" %>